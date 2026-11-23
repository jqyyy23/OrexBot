"""
hiwonder_bus.py — Python port of Hiwonder's official HX-30HM servo library.

The HX-30HM bus servos speak the Feetech STS (Dynamixel-style) protocol, NOT the
older LSC/LewanSoul 0x55 frame format:

    0xFF 0xFF  ID  LENGTH  CMD  PARAMS...  CHECKSUM

    LENGTH   = len(params) + 2
    CHECKSUM = (~(ID + LENGTH + CMD + PARAMS)) & 0xFF      (sum from ID onward)
    endianness: little-endian — low byte first

This mirrors the C++ reference `SerialServo` class in
`orex_head_control/Drone tracker/lib/Hiwonder/` (HX_30HM.cpp / .h / _Def.h) so the
two stay 1:1. Register addresses come straight from HX_30HM_Def.h.

IMPORTANT — signed positions:
    Present position / goal position / speed / offset are SIGN-MAGNITUDE, not
    plain integers. The reference uses:
        MASK_HOST(x, bit):  encode a signed host value -> servo word
        MASK_SERVO(x, bit): decode a servo word -> signed value
    e.g. present position uses bit 15 as the sign bit: a raw word 0x8000|N means
    the position is -N. Masking off the sign bit (0x7FFF) loses the sign and folds
    negative travel back onto positive — read_pos() below decodes it correctly.

Confirmed on this hardware: firmware 3.x, 1,000,000 baud, CH343 adapter enumerating
as /dev/ttyACM0. Arm IDs: 1=trigger, 2=gun pan, 3=shoulder lift, 4=shoulder rotate.
"""

import serial


# --- registers (from HX_30HM_Def.h) ----------------------------------------
# NVS (persist across power-cycle; write-protect at REG must be cleared first)
REG_ID                 = 5
REG_BAUD_RATE          = 6
REG_POS_OFFSET_L       = 31
REG_MODE               = 33
# read-write (SRAM)
REG_TORQUE_ENABLE      = 40
REG_ACC                = 41
REG_GOAL_POSITION_L    = 42
REG_PWM_SPEED_L        = 44
REG_GOAL_SPEED_L       = 46
REG_MAX_TORQUE_L       = 48
# read-only
REG_PRESENT_POSITION_L = 56
REG_PRESENT_SPEED_L    = 58
REG_PRESENT_LOAD_L     = 60
REG_PRESENT_VOLTAGE    = 62
REG_PRESENT_TEMPERATURE = 63
REG_MOVING_STATUS      = 66
REG_PRESENT_CURRENT_L  = 69

# --- instructions ----------------------------------------------------------
CMD_PING       = 1
CMD_READ       = 2
CMD_WRITE      = 3
CMD_REG_WRITE  = 4
CMD_ACTION     = 5
CMD_RESET      = 6
CMD_SYNC_READ  = 130
CMD_SYNC_WRITE = 131

BROADCAST_ID = 0xFE

# --- working modes (REG_MODE) ----------------------------------------------
POSITION_MODE          = 0
CLOSED_LOOP_MOTOR_MODE = 1   # closed-loop velocity — what the arm runs in
OPEN_LOOP_MOTOR_MODE   = 2

FRAME_HEADER = b"\xff\xff"


# --- sign-magnitude helpers (mirror the C++ MASK_HOST / MASK_SERVO macros) --
def mask_host(value, sign_bit):
    """Encode a signed host value into the servo's sign-magnitude word."""
    return ((-value) | (1 << sign_bit)) if value < 0 else value


def mask_servo(word, sign_bit):
    """Decode a servo sign-magnitude word into a signed value."""
    return -(word & ~(1 << sign_bit)) if (word & (1 << sign_bit)) else word


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class SerialServo:
    """
    Half-duplex driver for HX-30HM bus servos on one UART.

    Reads return the decoded value (int) or None on timeout / bad checksum.
    Writes return True on ack, False otherwise.
    """

    def __init__(self, port="/dev/ttyACM0", baudrate=1_000_000, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ------------------------------------------------------------------
    # framing
    # ------------------------------------------------------------------
    @staticmethod
    def _checksum(body):
        # body = [id, length, cmd, params...]  (sum from ID onward, like data_check)
        return (~sum(body)) & 0xFF

    def _send(self, servo_id, cmd, params=()):
        params = list(params)
        body = [servo_id, len(params) + 2, cmd] + params
        frame = FRAME_HEADER + bytes(body + [self._checksum(body)])
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
        except serial.SerialException:
            pass   # transient hiccup; the matching _recv will return None

    def _recv(self, data_len):
        """
        Read one response frame: FF FF ID LENGTH STATUS <data_len bytes> CHECKSUM.
        Returns (servo_id, status, [data...]) or None on failure.
        """
        try:
            # sync to the 0xFF 0xFF header
            header = self.ser.read(2)
            while header and header != FRAME_HEADER:
                header = header[1:] + self.ser.read(1)
            if header != FRAME_HEADER:
                return None
            rest = self.ser.read(4 + data_len)   # id, length, status, data..., checksum
        except serial.SerialException:
            # transient USB/serial hiccup (common on WSL) — caller keeps last value
            return None
        if len(rest) < 4 + data_len:
            return None
        servo_id, length, status = rest[0], rest[1], rest[2]
        data = list(rest[3:3 + data_len])
        checksum = rest[3 + data_len]
        # length = data_len + 2; recompute checksum over [id, length, status, data]
        if checksum != self._checksum([servo_id, length, status] + data):
            return None
        return servo_id, status, data

    # ------------------------------------------------------------------
    # core instructions
    # ------------------------------------------------------------------
    def ping(self, servo_id):
        """True if the servo answers."""
        self._send(servo_id, CMD_PING)
        return self._recv(0) is not None

    def general_read(self, servo_id, addr, data_len):
        """Read data_len bytes from register addr. Returns list[int] or None."""
        self._send(servo_id, CMD_READ, [addr, data_len])
        resp = self._recv(data_len)
        return resp[2] if resp else None

    def general_write(self, servo_id, addr, data):
        """Write bytes starting at register addr. Returns True on ack."""
        self._send(servo_id, CMD_WRITE, [addr] + list(data))
        if servo_id == BROADCAST_ID:
            return True
        return self._recv(0) is not None

    # ------------------------------------------------------------------
    # 16-bit little-endian register helpers
    # ------------------------------------------------------------------
    def _write_word(self, servo_id, addr, word):
        return self.general_write(servo_id, addr, [word & 0xFF, (word >> 8) & 0xFF])

    def _read_word(self, servo_id, addr):
        d = self.general_read(servo_id, addr, 2)
        return None if d is None else (d[0] | (d[1] << 8))

    # ------------------------------------------------------------------
    # torque / mode / config
    # ------------------------------------------------------------------
    def enable_torque(self, servo_id):
        return self.general_write(servo_id, REG_TORQUE_ENABLE, [1])

    def disable_torque(self, servo_id):
        return self.general_write(servo_id, REG_TORQUE_ENABLE, [0])

    def cali_pos(self, servo_id):
        """Calibrate: set the current pose as CENTER (2048). (Writes torque-enable=128.)"""
        return self.general_write(servo_id, REG_TORQUE_ENABLE, [128])

    def select_mode(self, servo_id, mode):
        """0=position, 1=closed-loop velocity (motor), 2=open-loop PWM."""
        return self.general_write(servo_id, REG_MODE, [mode & 0xFF])

    def write_acc(self, servo_id, acc):
        return self.general_write(servo_id, REG_ACC, [_clamp(acc, 0, 254)])

    def write_max_torque(self, servo_id, torque):
        """Max torque, 0..1000 (per-mille of rated)."""
        return self._write_word(servo_id, REG_MAX_TORQUE_L, _clamp(torque, 0, 1000))

    def write_pos_offset(self, servo_id, offset):
        """Position offset, -2047..2047 (sign bit 11). NVS."""
        offset = _clamp(offset, -2047, 2047)
        return self._write_word(servo_id, REG_POS_OFFSET_L, mask_host(offset, 11))

    def read_pos_offset(self, servo_id):
        w = self._read_word(servo_id, REG_POS_OFFSET_L)
        return None if w is None else mask_servo(w, 11)

    # ------------------------------------------------------------------
    # motion commands
    # ------------------------------------------------------------------
    def write_pos(self, servo_id, pos):
        """Goal position, -30719..30719 (sign bit 15)."""
        pos = _clamp(pos, -30719, 30719)
        return self._write_word(servo_id, REG_GOAL_POSITION_L, mask_host(pos, 15))

    def write_speed(self, servo_id, speed):
        """Goal speed, -3400..3400 (sign bit 15). Used in closed-loop motor mode."""
        speed = _clamp(speed, -3400, 3400)
        return self._write_word(servo_id, REG_GOAL_SPEED_L, mask_host(speed, 15))

    def write_pwm_speed(self, servo_id, speed):
        """Open-loop PWM speed, -1000..1000 (sign bit 10)."""
        speed = _clamp(speed, -1000, 1000)
        return self._write_word(servo_id, REG_PWM_SPEED_L, mask_host(speed, 10))

    def write_pos_ex(self, servo_id, acc, speed, pos):
        """
        Set acc + goal position in one write (matches the C++ write_pos_ex layout:
        [acc, posL, posH, 0, 0, posL, posH] starting at REG_ACC).
        """
        acc = _clamp(acc, 0, 254)
        pos = _clamp(pos, -30719, 30719)
        p = mask_host(pos, 15)
        pl, ph = p & 0xFF, (p >> 8) & 0xFF
        return self.general_write(servo_id, REG_ACC, [acc, pl, ph, 0, 0, pl, ph])

    # ------------------------------------------------------------------
    # feedback reads (all return decoded ints, or None)
    # ------------------------------------------------------------------
    def read_pos(self, servo_id):
        """Present position, signed (sign bit 15). None on failure."""
        w = self._read_word(servo_id, REG_PRESENT_POSITION_L)
        return None if w is None else mask_servo(w, 15)

    def read_speed(self, servo_id):
        w = self._read_word(servo_id, REG_PRESENT_SPEED_L)
        return None if w is None else mask_servo(w, 15)

    def read_pos_speed(self, servo_id):
        """Read present position and speed in one packet. Returns (pos, speed) or None."""
        d = self.general_read(servo_id, REG_PRESENT_POSITION_L, 4)
        if d is None:
            return None
        pos = mask_servo(d[0] | (d[1] << 8), 15)
        speed = mask_servo(d[2] | (d[3] << 8), 15)
        return pos, speed

    def read_load(self, servo_id):
        """Present load, signed (sign bit 10), per-mille of rated torque."""
        w = self._read_word(servo_id, REG_PRESENT_LOAD_L)
        return None if w is None else mask_servo(w, 10)

    def read_voltage(self, servo_id):
        """Supply voltage in 0.1 V units (1 byte). e.g. 124 -> 12.4 V."""
        d = self.general_read(servo_id, REG_PRESENT_VOLTAGE, 1)
        return None if d is None else d[0]

    def read_temperature(self, servo_id):
        d = self.general_read(servo_id, REG_PRESENT_TEMPERATURE, 1)
        return None if d is None else d[0]

    def read_current(self, servo_id):
        """Present current in mA (unsigned word)."""
        return self._read_word(servo_id, REG_PRESENT_CURRENT_L)

    def read_moving_status(self, servo_id):
        d = self.general_read(servo_id, REG_MOVING_STATUS, 1)
        return None if d is None else d[0]

    # ------------------------------------------------------------------
    # sync (multi-servo) — efficient for reading/writing the whole arm at once
    # ------------------------------------------------------------------
    def sync_read_cur_pos_ex(self, ids):
        """
        Read present position/speed/load/voltage/temperature for several servos in
        one bus transaction (SYNC READ over 8 bytes from REG_PRESENT_POSITION_L).

        Returns {id: {"pos","speed","load","voltage","temp"}}; missing servos omitted.
        """
        ids = list(ids)
        byte_num = 8
        self._send(BROADCAST_ID, CMD_SYNC_READ,
                   [REG_PRESENT_POSITION_L, byte_num] + ids)
        out = {}
        for _ in ids:
            resp = self._recv(byte_num)
            if resp is None:
                continue
            sid, _status, d = resp
            if sid not in ids or sid in out:
                continue   # key by the frame's real ID so a slow servo can't misalign
            out[sid] = {
                "pos":     mask_servo(d[0] | (d[1] << 8), 15),
                "speed":   mask_servo(d[2] | (d[3] << 8), 15),
                "load":    mask_servo(d[4] | (d[5] << 8), 10),
                "voltage": d[6],
                "temp":    d[7],
            }
        return out

    def sync_write_pos_ex(self, commands):
        """
        Move several servos at once. `commands` = list of (id, acc, speed, pos).
        Mirrors the C++ sync_write_pos_ex 8-byte-per-servo layout
        ([id, acc, posL, posH, 0, 0, speedL, speedH]) via SYNC WRITE at REG_ACC.
        """
        buf = []
        for sid, acc, speed, pos in commands:
            acc = _clamp(acc, 0, 254)
            speed = _clamp(speed, -3400, 3400)
            pos = _clamp(pos, -30719, 30719)
            p = mask_host(pos, 15)
            s = mask_host(speed, 15)
            buf += [sid, acc, p & 0xFF, (p >> 8) & 0xFF,
                    0, 0, s & 0xFF, (s >> 8) & 0xFF]
        # SYNC WRITE: [addr, param_len_per_servo, id1, <7 bytes>, id2, <7 bytes>, ...]
        self._send(BROADCAST_ID, CMD_SYNC_WRITE, [REG_ACC, 7] + buf)
        return True   # broadcast: no ack


if __name__ == "__main__":
    # Quick smoke test: ping + read the arm joints.
    bus = SerialServo("/dev/ttyACM0")
    for sid in (2, 3, 4):
        if bus.ping(sid):
            print(f"ID {sid}: pos={bus.read_pos(sid)}  "
                  f"volt={bus.read_voltage(sid)/10:.1f}V  temp={bus.read_temperature(sid)}C")
        else:
            print(f"ID {sid}: no response")
    print("sync:", bus.sync_read_cur_pos_ex([2, 3, 4]))
    bus.close()
