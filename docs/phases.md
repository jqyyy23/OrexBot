# OrexBot Assembly Timeline & Progress

**Project Goal:** Assemble the OrexBot open-source arm (based on RX1 from Red-Rabbit-Robotics) for dishwashing demonstrations.

**Reference:** https://github.com/Red-Rabbit-Robotics | **Software:** ROS1 (for quick demo), planning ROS2 migration

---

## Timeline: 3 Full Days (Friday–Sunday Night)

**Status:** Thursday night — logging off for the night  
**Deadline:** Sunday night — full assembly complete

---

## Current Progress

### Printing Status
- ✅ Most parts printed
- ⏳ Remaining: Small gears (~20 min print time), 1 plate, clamps

### Assembly Status
- ✅ 1 out of 8 gearboxes assembled
- ✅ Main pole ~50% complete
- ⏳ Need: 7 more gearboxes, pole completion
- ⏳ Servo teardown: 2 more from one LeRobot arm, 6 from the other LeRobot arm

### Design Preparation
- ✅ All components reviewed and modified for aluminum pole + salvaged metal tubing
- ⏳ Fit testing needed before full assembly

---

## Assembly Plan (Friday–Sunday)

### Friday
- [ ] Finish printing remaining parts (gears, plate, clamps)
- [ ] Complete main pole assembly
- [ ] Begin gearbox assembly (target: 4–5 complete)
- [ ] Start servo teardown from LeRobot arms

### Saturday
- [ ] Finish remaining gearbox assemblies (target: all 8 complete)
- [ ] Complete servo teardown
- [ ] Fit testing of all arm components
- [ ] Build arm from shoulder down, testing as you go

### Sunday
- [ ] Final assembly pass
- [ ] Hand assembly (if time permits)
- [ ] Full system test
- [ ] **Deadline: Assembly complete by Sunday night**

---

## Blockers & Constraints

### ⚠️ Teleop Structure (NOT Realistic)
- **Issue:** Missing encoders and servos needed for teleop hardware
- **Decision:** Deprioritize teleop for this sprint; focus on basic arm assembly and demo
- **Action Item:** 🔴 **Talk to Miti about scrap servos** — see if any can be salvaged or borrowed

### Hands
- Not included in this sprint
- Can be integrated post-assembly if time allows

---

## Build Strategy

1. **Shoulder → Down Assembly:** Build the arm progressively from the shoulder, testing fit at each stage
2. **Incremental Testing:** Validate each section before moving to the next
3. **Software:** Start with ROS1 for quick demo; plan ROS2 migration for learning