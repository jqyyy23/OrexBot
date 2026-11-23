## Plan: Head Vision Depth Tracker

Build a ROS 2 `ament_python` vision node for the drone tracker that uses RealSense RGB+depth, YOLO detection, D435i IMU ego-motion compensation, and a simple center-based tracking path. For now, keep the scope host-side only and make the package runnable either standalone or from the main bringup later, with a clean internal output contract that can be adapted to the ESP32 in a later pass.

**Current milestone**
- Step 1 is complete: the host-side runtime contract is defined around the RealSense camera topics and the head-control output topic .
- Step 2 is complete: the `orex_head_vision` ROS package has been scaffolded and built successfully .
- Step 3 is effectively complete for the vision streams: RGB, depth, and camera-info streams are being subscribed and synchronized with `ApproximateTimeSynchronizer`. IMU subscription is being added.
- Step 4 is complete: YOLO inference is running on live RGB frames, the best detection is selected, the center pixel is extracted, and the aligned depth is sampled at that location .
- Step 5 is **implemented**: the node subscribes to the unified `/camera/camera/imu` topic (`unite_imu_method:=1`), integrates the gyro via strapdown/Rodrigues into a running camera->reference rotation `R_cur`, and rotates each deprojected point into that fixed reference frame so head pan/tilt no longer registers as target velocity. Outputs are rotated back into `camera_color_optical_frame`. *Pending live verification (step 3 below) and a possible gyro-sign flip.*
- Step 6 is **implemented and hardened**: 6D constant-velocity Kalman filter on the ego-compensated point, plus a coast timeout (`COAST_TIMEOUT_S = 0.5`) that declares the target lost, halts output, and resets the filter for a clean re-acquire. Lock state is published on `/head/target_locked` (`std_msgs/Bool`).
- Step 7 is **implemented**: the filtered position is rotated back into the current camera frame and converted to yaw/pitch angular error on `/head/angular_error`.
- **[NEW] Tier 1 track-before-detect is implemented**: when locked, YOLO runs on a crop centered on the filter's predicted pixel at a lower confidence (`ROI_CONF = 0.2`), with a window that grows per consecutive miss; otherwise it searches the full frame at `FULL_CONF = 0.4`. This attacks the root cause of coasting (missed detections) rather than only extrapolating through them.
- An IMU health check timer warns loudly if the gyro stream never arrives or stalls (ego-motion silently degrades to identity otherwise).
- **[NEW] Deployment split:** `vision_tracking` runs headless (detect -> deproject -> ego-comp -> KF -> publish only). A separate `turret_view` HUD node consumes the track topics and renders the turret cam on demand. Output topics: `/head/target_point` (PointStamped), `/head/angular_error` (Vector3, yaw/pitch rad), `/head/target_velocity` (Vector3, m/s camera frame), `/head/target_locked` (Bool).
- Step 8 remains a later refinement: launch/runtime configuration .
- Step 9 is the next verification milestone: confirm the live ROS topic outputs and runtime behavior end to end .

**Steps**
1. Define the host-side runtime contract: which camera namespace to read, which topic to publish for downstream control, and which YOLO class labels will act as the temporary drone proxy. *Completed.* 
2. Scaffold the `orex_head_vision` ROS package with `ament_python` metadata, entry points, and the executable node class so it can run standalone or be launched by a higher-level bringup. *Completed.* 
3. Implement synchronized RGB + depth + camera info ingestion and intrinsics handling with message filters, using approximate synchronization as the default for bridge/WSL timing jitter. *Completed for vision streams.* 
4. Add YOLO inference and extract the target center pixel, then deproject the detection into 3D using aligned depth. *Completed.* 
5. Subscribe to the RealSense D435i IMU stream (`sensor_msgs/Imu`). Extract the angular velocity (gyro) and calculate the camera's rotational shift over `dt`. Apply the inverse rotation to the deprojected 3D point to eliminate artificial ego-motion caused by the pan/tilt head. *Implemented (`imu_callback`, strapdown/Rodrigues into `R_cur`); pending live verification.*
6. Implement the 6D constant-velocity Kalman filter on the stabilized 3D coordinates, utilizing timestamp-based `dt`, prediction, correction, and coast-only behavior on missed frames. *Implemented, plus coast timeout + `/head/target_locked` lock-state output.*
7. Convert the stabilized, filtered 3D point into yaw/pitch angular error outputs and publish the result on the host-side command topic. *Implemented.*
8. **[NEW] Robustness — reject distractors and reduce dropouts:**
   - **Data association + validation gate (done):** every detection is deprojected to 3D; a detection farther than `GATE_BASE_M (+ GATE_GROW_M * misses)` from the filter's prediction is rejected as a distractor and treated as a miss (coast) instead of snapping the track. Among in-gate candidates the one *closest to the prediction* is chosen, not merely the most confident. Depth is sampled as a windowed median (`DEPTH_WIN_HALF`) to reject single-pixel outliers. This fixes the "locked but jumps to random indoor objects" failure.
   - **Track confirmation (done):** `LOCKED` is only declared after `CONFIRM_HITS` consecutive in-gate detections, so a one-frame false positive stays `SEARCHING`; the lock then latches through the coast window.
   - **Tier 1 track-before-detect (done):** ROI crop around the predicted pixel + adaptive confidence (`ROI_CONF`/`FULL_CONF`), window grows per consecutive miss. **Velocity-aware (done):** the ROI is centered on the velocity-led prediction (`pos + vel*dt`, consistent with the gate) and its half-size adds a capped share (`ROI_VEL_GAIN`) of the predicted pixel displacement as uncertainty margin, bounded by `ROI_MAX_HALF_FRAC` so the zoom-in benefit is preserved. Sizing on prediction *uncertainty* rather than raw velocity keeps a smoothly-moving target's window tight.
   - **Tier 2 (next):** bridge gaps with an image-appearance tracker (OpenCV CSRT/KCF or ByteTrack-style association) so the target is followed *through* YOLO misses and can follow turns.
   - **Tier 3 (if needed):** upgrade the motion model from constant-velocity to an IMM (CV + coordinated-turn + CA) for maneuvering targets.
   - **Possible refinement:** replace the fixed metric gate with a statistical Mahalanobis gate using the KF innovation covariance `S = H P H^T + R`.
9. Integrate launch/runtime configuration for the RealSense namespace, IMU filter nodes, model path, detection class list, and tuning parameters. 
10. Verify with ROS build/type checks and a runtime smoke test on recorded or live camera/IMU topics. 

**Relevant files**
- `/home/jay/github/OrexBot/src/drone_tracker_plan.md` - source specification to implement .
- `/home/jay/github/OrexBot/src/orex_head_control/Drone tracker/src/main.cpp` - current ESP32 subscriber contract and unit expectations for later integration .
- `/home/jay/github/OrexBot/src/orex_head_control/stack overview.md` - existing system topology and topic naming reference .
- `/home/jay/github/OrexBot/src/orex_description/launch/orexbot.launch.py` - current ROS launch style and package structure reference .

**Verification**
1. Check package build/install behavior with `colcon build` for the new ROS package .
2. Run a focused ROS topic smoke test to confirm synchronized image/depth subscription and published host-side tracking output .
3. Validate ego-motion compensation by moving the camera head while tracking a stationary target; the filtered output should not register significant velocity. *If the logged `ego-comp velocity` doubles instead of cancelling, negate `rotvec` in `imu_callback` (gyro axis sign).*
4. Validate coast + lock: occlude/remove the target briefly and confirm `/head/target_point` and `/head/angular_error` keep publishing during the coast window, then `/head/target_locked` flips to `False` and output halts after `COAST_TIMEOUT_S`; re-acquisition re-initializes cleanly.
5. Validate Tier 1 ROI: confirm `ROI-gated inference` log lines appear while locked, and that a small/distant target held near the last position stays detected at the lower ROI confidence.
5b. Validate gating/confirmation: with clutter around, confirm the track does not jump to random objects (look for `Gate rejected N detection(s)` logs), that `LOCKED` only appears after a few consistent detections, and that waving a distractor near the boresight does not steal the lock.
6. Confirm the IMU health warning fires when the camera is launched without `unite_imu_method:=1`.
7. Confirm the launch path works both standalone and when included from the larger robot bringup .

**Decisions**
- Use approximate time synchronization rather than exact synchronization, because it is more tolerant of RealSense and bridge jitter .
- Use the drone-specific Hugging Face YOLOv8n model as the starting detector, with its `quadcopter` and `fixed-wing` classes, then swap in a custom drone dataset later .
- For this pass, stabilize detection by choosing one track and extracting its center pixel . 
- Use the RealSense D435i's internal 6-axis IMU for ego-motion compensation rather than strict fixed-frame TF transformations, keeping the vision node decoupled from the robot's URDF and physical kinematics.
- Defer the ESP32 topic/unit migration until the head-tracking node is stable .
- Keep the package runnable standalone first, then optionally wire it into the full bringup later .

**Development approach**
- Build one ROS 2 concept per milestone so each step is understandable and testable .
- Start with a minimal node that only subscribes and republishes a diagnostic message, then add camera sync, then detection, then deprojection, then IMU compensation, then optional smoothing .
- Prefer small runtime smoke tests after each step so failures stay local and easy to debug .
- Run the Intel RealSense hardware calibration tool on the D435i to save IMU intrinsic zero-offsets directly to the device's NVRAM to minimize baseline gyro drift.
- If necessary, route the raw IMU stream through a lightweight software filter (like `imu_filter_madgwick`) to smooth gyro noise before the ego-motion math.
- **[Done] Visualization is split into a separate on-demand `turret_view` HUD node** (`ros2 run orex_head_vision turret_view`). The `vision_tracking` node now runs fully headless — no `plot()`, colormap, `imshow`, or per-frame `imwrite` on the inference thread. `turret_view` is a pure consumer of the published track topics and draws a lightweight turret overlay: a boresight crosshair, a corner-bracket target reticle at the reprojected `/head/target_point`, a predicted-motion arrow from `/head/target_velocity`, and a `LOCKED`/`SEARCHING` status + range readout driven by `/head/target_locked`. It can be opened/closed anytime without affecting tracking (close with `q`/ESC).
- Keep the visualization lightweight by avoiding full-frame debug images, large overlays, or image saving on every callback; use only minimal drawing and only when it helps monitoring or demos . *Enforced by the headless-node / separate-HUD split above.*
- Consider switching to IPC with camera pub and sub to reduce latency later on