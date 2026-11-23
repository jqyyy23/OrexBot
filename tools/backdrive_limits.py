#!/usr/bin/env python3
"""
backdrive_limits.py — find joint limits on the HX-30HM gun arm by hand.

Disables torque on the given servo(s) so each joint can be back-driven by hand,
then live-tracks the min/max encoder position you reach as you sweep each joint
to its mechanical stops. Press Ctrl+C when done: it prints the limits and
RE-ENABLES torque on every servo it touched (including on error).

--- Protocol notes (VERIFIED against the actual hardware) --------------------
The HX-30HM speaks the Feetech STS/SCS protocol: half-duplex UART, 1 Mbps,
0xFF 0xFF header, little-endian, checksum = ~sum & 0xFF. Instructions:
0x01 ping, 0x02 read, 0x03 write, 0x83 sync-write.

The register map is the standard Feetech STS layout, NOT the layout in Hiwonder's
published "V3.7 / firmware 2.43" doc (these servos run firmware 3.11, whose map
differs). Confirmed live on 2026-07-27:
  * 0x28 Torque Enable  — 1 byte, SRAM: 0 = torque OFF (free), 1 = ON, 128 = damping.
                          SRAM = volatile, no NVS unlock, cleared on power loss.
  * 0x38 Present Position — 2 bytes, read-only, 0-4095 over 360deg (~0.088deg/step).
  * 0x3E Present Voltage  — 1 byte, 0.1 V.   0x3F Present Temperature — 1 byte, degC.
  * 0x10 Max Torque (2 bytes) is the real torque limit; 0x13 in the old doc is junk.

Because torque is a plain SRAM write, this script makes NO non-volatile changes:
worst case (hard kill), power-cycle the servos and torque returns to its saved state.

Usage (this arm: ID2=gun pan, ID3=shoulder lift, ID4=shoulder rotate):
    python3 tools/backdrive_limits.py --port /dev/ttyACM0 --ids 4 3 2 \
        --names gun_shoulder_rotate gun_shoulder_lift gun_pan

    # one joint at a time is easier to sweep cleanly:
    python3 tools/backdrive_limits.py --port /dev/ttyACM0 --ids 3 --names gun_shoulder_lift

SAFETY: freeing gun_shoulder_lift (ID3) removes the torque holding the arm up —
support the arm by hand before/while it is free, or it will drop under gravity.

Requires: pyserial  (pip install pyserial)
"""

from __future__ import annotations
import argparse
import sys
import time

import serial


# --- HX-30HM / Feetech STS register map (only what we use) -----------------
REG_POS_OFFSET    = 0x1F   # 2 bytes, EEPROM: sign-magnitude, bit 11 = sign, mag 0-2047
REG_TORQUE_ENABLE = 0x28   # 1 byte, SRAM: 0 = off/free, 1 = on, 128 = calibrate-to-center(2048)
REG_NVS_LOCK      = 0x37   # 1 byte: 0 = unlocked, 1 = locked (needed to write EEPROM regs)
REG_CURRENT_POS   = 0x38   # 2 bytes, read-only
REG_VOLTAGE       = 0x3E   # 1 byte, 0.1 V
REG_TEMP          = 0x3F   # 1 byte, degC

TORQUE_OFF = 0
TORQUE_ON  = 1

# Present position (0x38): bit 15 is a rotation-DIRECTION flag (set while turning
# one way), NOT part of the value — unmasked, 2012 reads as 0x8000|2012 = 34780
# while moving. Mask 0x7FFF to strip it while keeping the value CONTINUOUS.
#
# We deliberately do NOT wrap to 12 bits (0x0FFF): present = magnetic_angle - offset
# and does not wrap in the register, so with a shifting offset (e.g. the -2047 set to
# keep a joint's travel off the encoder seam) present intentionally runs past 4095
# (up to ~6142). Keeping it continuous means min/max span the real travel; wrapping
# to 0x0FFF would inject a false seam at present=4096 mid-range. The absolute number
# is offset-shifted and may exceed 360° — only the span (max-min) is meaningful here.
POS_MASK        = 0x7FFF
OFFSET_SIGN_BIT = 0x800    # bit 11 marks a negative offset
OFFSET_MAG_MAX  = 0x7FF    # magnitude is 11 bits (0-2047)

BROADCAST_ID    = 0xFE
STEPS_PER_REV   = 4096
DEG_PER_STEP    = 360.0 / STEPS_PER_REV   # ~0.0879 deg/step


class ServoError(Exception):
    pass


class HiwonderBus:
    """Minimal half-duplex driver for HX-30HM (Feetech STS) bus servos."""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.05):
        self._ser = serial.Serial(
            port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def close(self):
        self._ser.close()

    @staticmethod
    def _checksum(payload: list[int]) -> int:
        return (~sum(payload)) & 0xFF

    def _build_packet(self, servo_id: int, instruction: int, params: list[int]) -> bytes:
        length = len(params) + 2
        payload = [servo_id, length, instruction] + params
        cs = self._checksum(payload)
        return bytes([0xFF, 0xFF] + payload + [cs])

    def _send(self, packet: bytes):
        self._ser.reset_input_buffer()
        self._ser.write(packet)

    def _recv(self, expected_params: int) -> tuple[int, int, list[int]]:
        """Read one response packet. Returns (servo_id, error, params)."""
        buf = b""
        deadline = time.monotonic() + self._ser.timeout
        while time.monotonic() < deadline:
            b = self._ser.read(1)
            if not b:
                break
            buf += b
            if len(buf) >= 2 and buf[-2:] == b"\xff\xff":
                break

        rest = self._ser.read(4 + expected_params)
        if len(rest) < 4:
            raise ServoError("timeout: no response")

        servo_id = rest[0]
        length   = rest[1]
        error    = rest[2]
        params   = list(rest[3 : 3 + length - 2])
        checksum = rest[3 + length - 2] if len(rest) > 3 + length - 2 else 0

        payload = [servo_id, length, error] + params
        if self._checksum(payload) != checksum:
            raise ServoError(f"checksum mismatch (id={servo_id})")
        if error:
            raise ServoError(f"servo {servo_id} error flags: 0x{error:02X}")
        return servo_id, error, params

    def ping(self, servo_id: int) -> bool:
        self._send(self._build_packet(servo_id, 0x01, []))
        try:
            self._recv(0)
            return True
        except ServoError:
            return False

    def read(self, servo_id: int, start_addr: int, length: int) -> list[int]:
        self._send(self._build_packet(servo_id, 0x02, [start_addr, length]))
        _, _, params = self._recv(length)
        return params

    def write(self, servo_id: int, start_addr: int, data: list[int]):
        self._send(self._build_packet(servo_id, 0x03, [start_addr] + data))
        if servo_id != BROADCAST_ID:
            self._recv(0)

    def read_byte(self, servo_id: int, addr: int) -> int:
        return self.read(servo_id, addr, 1)[0]

    def read_word(self, servo_id: int, addr: int) -> int:
        d = self.read(servo_id, addr, 2)
        return d[0] | (d[1] << 8)


# --- torque helpers (plain SRAM writes — no NVS involved) ------------------
def get_torque(bus: HiwonderBus, servo_id: int) -> int:
    return bus.read_byte(servo_id, REG_TORQUE_ENABLE)


def set_torque(bus: HiwonderBus, servo_id: int, value: int):
    bus.write(servo_id, REG_TORQUE_ENABLE, [value & 0xFF])


def read_position(bus: HiwonderBus, servo_id: int) -> int:
    """Present position as a clean 12-bit angle (0-4095)."""
    return bus.read_word(servo_id, REG_CURRENT_POS) & POS_MASK


# --- position-offset / re-zero helpers -------------------------------------
def get_offset(bus: HiwonderBus, servo_id: int) -> int:
    """Read the position-correction offset as a signed value (-2047..2047)."""
    raw = bus.read_word(servo_id, REG_POS_OFFSET)
    mag = raw & OFFSET_MAG_MAX
    return -mag if (raw & OFFSET_SIGN_BIT) else mag


def _encode_offset(signed: int) -> int:
    signed = max(-OFFSET_MAG_MAX, min(OFFSET_MAG_MAX, signed))
    return (abs(signed) & OFFSET_MAG_MAX) | (OFFSET_SIGN_BIT if signed < 0 else 0)


def set_current_to_zero(bus: HiwonderBus, servo_id: int) -> tuple[int, int, int, int]:
    """
    Make the servo report position 0 at its current pose, by writing the offset
    register (0x1F). present = magnetic_angle - offset, so offset := magnetic_angle
    (chosen in the servo's [-2047, 2047] sign-magnitude range, mod 4096).

    Returns (old_offset, magnetic_angle, new_offset, present_after).
    """
    old_off = get_offset(bus, servo_id)
    present = read_position(bus, servo_id)
    angle = (present + old_off) % STEPS_PER_REV          # true magnetic angle 0-4095
    new_signed = angle if angle <= OFFSET_MAG_MAX else angle - STEPS_PER_REV

    # Offset lives in EEPROM — clear the write-lock, write, restore the lock.
    enc = _encode_offset(new_signed)
    orig_lock = bus.read_byte(servo_id, REG_NVS_LOCK)
    if orig_lock:
        bus.write(servo_id, REG_NVS_LOCK, [0])
    bus.write(servo_id, REG_POS_OFFSET, [enc & 0xFF, (enc >> 8) & 0xFF])
    if orig_lock:
        bus.write(servo_id, REG_NVS_LOCK, [orig_lock])

    time.sleep(0.1)   # let the new offset propagate to the present-position register
    return old_off, angle, new_signed, read_position(bus, servo_id)


def steps_to_deg(steps: int) -> float:
    return steps * DEG_PER_STEP


def main():
    ap = argparse.ArgumentParser(
        description="Back-drive HX-30HM joints with torque off to find their limits.")
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial port of the bus adapter (default: /dev/ttyACM0, the CH343)")
    ap.add_argument("--ids", type=int, nargs="+", default=[4, 3, 2],
                    help="servo ID(s) to free and track (default: 4 3 2)")
    ap.add_argument("--names", nargs="+", default=None,
                    help="optional joint names, aligned to --ids "
                         "(e.g. gun_shoulder_rotate gun_shoulder_lift gun_pan)")
    ap.add_argument("--baud", type=int, default=1_000_000,
                    help="baud rate (default: 1000000, the HX-30HM factory default)")
    ap.add_argument("--hz", type=float, default=20.0,
                    help="live refresh rate (default: 20 Hz)")
    ap.add_argument("--zero", action="store_true",
                    help="set each --ids servo's CURRENT pose to read position 0 "
                         "(writes the offset register, persists across power-cycle), "
                         "then exit without back-driving")
    args = ap.parse_args()

    names = {}
    if args.names:
        if len(args.names) != len(args.ids):
            ap.error("--names must have the same count as --ids")
        names = dict(zip(args.ids, args.names))

    try:
        bus = HiwonderBus(args.port, baudrate=args.baud)
    except serial.SerialException as e:
        print(f"ERROR: could not open {args.port}: {e}", file=sys.stderr)
        print("  Check the adapter is connected and (on WSL) attached via usbipd.",
              file=sys.stderr)
        return 1

    # --- re-zero mode: shift each servo's zero to its current pose, then exit ---
    if args.zero:
        try:
            print("Setting current pose to position 0 (writes offset register):\n")
            for sid in args.ids:
                if not bus.ping(sid):
                    print(f"  servo {sid}: no response — skipping", file=sys.stderr)
                    continue
                old_off, angle, new_off, present = set_current_to_zero(bus, sid)
                label = names.get(sid, "?")
                ok = "OK" if min(present, STEPS_PER_REV - present) <= 3 else "CHECK"
                print(f"  servo {sid} ({label}): angle={angle}  offset {old_off} -> {new_off}  "
                      f"now reads {present} [{ok}]")
            print("\nDone. Re-measure joint limits after zeroing — the encoder steps have shifted.")
        finally:
            bus.close()
        return 0

    original_torque: dict[int, int] = {}   # ids we freed -> torque value to restore
    lo: dict[int, int] = {}
    hi: dict[int, int] = {}

    try:
        # --- confirm servos, save torque state, disable torque ---
        for sid in args.ids:
            if not bus.ping(sid):
                print(f"WARNING: servo ID {sid} did not respond — skipping.", file=sys.stderr)
                continue
            original_torque[sid] = get_torque(bus, sid)
            set_torque(bus, sid, TORQUE_OFF)
            volt = bus.read_byte(sid, REG_VOLTAGE) * 0.1
            print(f"servo {sid} ({names.get(sid, '?')}): torque OFF "
                  f"(was {original_torque[sid]}), {volt:.1f} V — free to back-drive")

        active = list(original_torque.keys())
        if not active:
            print("No servos responded. Nothing to do.", file=sys.stderr)
            return 1

        # seed limits with the current position
        for sid in active:
            pos = read_position(bus, sid)
            lo[sid] = hi[sid] = pos

        print()
        if 3 in active:
            print("!! ID3 (shoulder lift) is now free — SUPPORT THE ARM so it doesn't drop.\n")
        print("Slowly sweep each joint to BOTH mechanical stops.")
        print("Min/max update live below. Press Ctrl+C when done to print limits "
              "and re-enable torque.\n")

        period = 1.0 / args.hz
        printed_lines = 0
        cur: dict[int, int] = dict.fromkeys(active, 0)
        while True:
            for sid in active:
                try:
                    pos = read_position(bus, sid)
                except ServoError:
                    continue   # transient bus hiccup; keep going
                cur[sid] = pos
                if pos < lo[sid]:
                    lo[sid] = pos
                if pos > hi[sid]:
                    hi[sid] = pos

            # redraw the live block in place
            if printed_lines:
                sys.stdout.write(f"\033[{printed_lines}A")
            lines = []
            for sid in active:
                label = names.get(sid, f"id {sid}")
                span = hi[sid] - lo[sid]
                lines.append(
                    f"  {label:<22} now {cur[sid]:4d} ({steps_to_deg(cur[sid]):6.1f}°)  "
                    f"min {lo[sid]:4d} ({steps_to_deg(lo[sid]):6.1f}°)  "
                    f"max {hi[sid]:4d} ({steps_to_deg(hi[sid]):6.1f}°)  "
                    f"range {span:4d} ({steps_to_deg(span):5.1f}°)"
                )
            block = "\n".join(f"\033[2K{ln}" for ln in lines)
            sys.stdout.write(block + "\n")
            sys.stdout.flush()
            printed_lines = len(lines)

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n\nDone sweeping. Limits found:\n")
        for sid in original_torque:
            label = names.get(sid, f"id {sid}")
            span = hi[sid] - lo[sid]
            print(f"  {label} (servo {sid}):")
            print(f"    min = {lo[sid]:4d} steps  ({steps_to_deg(lo[sid]):7.2f}°)")
            print(f"    max = {hi[sid]:4d} steps  ({steps_to_deg(hi[sid]):7.2f}°)")
            print(f"    range = {span} steps ({steps_to_deg(span):.2f}°)")
            if lo[sid] <= 2 or hi[sid] >= STEPS_PER_REV - 3:
                print("    NOTE: range touches the 0/4095 encoder wrap — this joint may "
                      "straddle the zero-crossing, which corrupts min/max. If the travel is "
                      "really one continuous sweep through 0/4095, re-zero the servo so its "
                      "range sits away from the wrap, then re-run.")
            print()
    finally:
        # ALWAYS restore each servo to the torque state we found it in.
        for sid, val in original_torque.items():
            try:
                set_torque(bus, sid, val)
                print(f"servo {sid}: torque restored to {val}")
            except ServoError as e:
                print(f"servo {sid}: FAILED to restore torque ({e}) — "
                      f"power-cycle before use!", file=sys.stderr)
        bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
