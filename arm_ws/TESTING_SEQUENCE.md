# Testing Sequence

This workspace exposes a simple motion pipeline:

`/imu/accel` -> `controller` -> `/controller/displacement` -> `ik_solver` -> `/joint_states`

Use the sequence below to test each stage in isolation before using the live IMU.

## 1. Build and source

```bash
cd /home/anant/gesture_controlled_arm/arm_ws
colcon build
source /opt/ros/humble/setup.bash
source /home/anant/gesture_controlled_arm/arm_ws/install/setup.bash
```

## 2. Start the manual test stack

This launch file starts `robot_state_publisher`, `controller`, `ik_solver`, the static transform, and RViz.
It skips the serial IMU reader by default, so it is safe to use without hardware.

```bash
ros2 launch imu_reader manual_test.launch.py
```

Optional:

```bash
ros2 launch imu_reader manual_test.launch.py use_rviz:=false
ros2 launch imu_reader manual_test.launch.py use_imu_reader:=true
```

## 3. Confirm topics are alive

```bash
ros2 topic list | grep -E 'imu|joint_states|controller/displacement'
ros2 topic echo /joint_states
```

Expected result:

- `ik_solver` should publish an initial `/joint_states` message.
- `controller` should be subscribed to `/imu/accel`.
- `ik_solver` should be subscribed to `/controller/displacement`.

## 4. Test IK solver directly

This bypasses the controller and verifies that displacement commands move the arm.

```bash
cd /home/anant/gesture_controlled_arm
source /opt/ros/humble/setup.bash
source /home/anant/gesture_controlled_arm/arm_ws/install/setup.bash
./arm_ws/scripts/publish_test_sequence.sh ik
```

What to watch:

- RViz arm pose should step through several reachable positions.
- `/joint_states` values should change after each displacement command.
- `ik_solver` logs should show `Applied displacement=...`.

## 5. Test controller to IK path

This feeds synthetic acceleration samples into `/imu/accel`.

```bash
cd /home/anant/gesture_controlled_arm
source /opt/ros/humble/setup.bash
source /home/anant/gesture_controlled_arm/arm_ws/install/setup.bash
./arm_ws/scripts/publish_test_sequence.sh controller
```

What to watch:

- `controller` logs should show changing `lin=`, `cmd=`, and `disp=` fields.
- `ik_solver` should receive the resulting displacements and publish updated joint states.
- RViz should show smaller, smoother motion than the direct IK test.

## 6. Full hardware test

When the IMU is connected and streaming on the expected serial port:

```bash
ros2 launch imu_reader manual_test.launch.py use_imu_reader:=true
```

Then move the IMU slowly first, then with larger gestures, and watch:

- `/imu/accel` for live sensor data
- controller logs for threshold engagement
- `/joint_states` and RViz for arm motion

## 7. Useful debug commands

```bash
ros2 node list
ros2 topic hz /imu/accel
ros2 topic hz /controller/displacement
ros2 topic hz /joint_states
ros2 topic echo /controller/displacement
```

## 8. Failure checklist

- If RViz does not move during the direct IK test, inspect `ik_solver` logs first.
- If the controller test shows no motion, lower `motion_engage_threshold` or increase the synthetic acceleration values.
- If the hardware test fails but the controller test passes, the issue is likely in serial input or IMU packet formatting.
