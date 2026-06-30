# CLAUDE.md

AcadBot — the hands-on code project for the **Robotics Academy SLAM track**
(Summer 2026, Sessions 12–15). A simulated differential-drive robot that students
take from raw-odometry drift → LiDAR SLAM → visual SLAM → full autonomous
navigation. Everything runs in **ROS 2 Jazzy + Gazebo Harmonic** inside Docker.

## Layout
- `docker/` — `Dockerfile`, `user_install.sh` (deps), `build_ros_image.sh`, `run_ros_container.sh`
- `ros2_ws/src/` — six packages:
  - `acadbot_description` — URDF/xacro robot + sensors (2D LiDAR, RGBD cam, IMU), RViz configs
  - `acadbot_gazebo` — Gazebo world, robot spawn, `ros_gz` bridge
  - `acadbot_control` — **C++** nodes: `square_driver` (drift demo), `patrol_commander` (Nav2 action client)
  - `acadbot_slam` — `slam_toolbox` mapping + localization
  - `acadbot_navigation` — Nav2 params (incl. recovery behaviors), maps
  - `acadbot_bringup` — per-session one-command launch files
- `README.md` / `PROJECT.md` — student-facing guide + architecture.

## Build & run
```bash
cd docker && ./build_ros_image.sh          # build image  acadbot:jazzy
./run_ros_container.sh                      # shell; mounts ../ros2_ws at /ros2_ws
# inside the container:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## Key launches (per session)
- S1 drift: `ros2 launch acadbot_gazebo simulation.launch.py` + `acadbot_control square_driver.launch.py`
- S2 SLAM: `ros2 launch acadbot_bringup mapping.launch.py`
- S4 autonomy: `ros2 launch acadbot_bringup autonomy.launch.py` + `acadbot_control patrol.launch.py`
- Headless (no GPU/CI): add `headless:=true` to `simulation.launch.py`.

## Conventions / gotchas
- ROS 2 **Jazzy** ⇒ Gazebo **Harmonic** (gz-sim 8); sensor/plugin syntax is the new `gz-sim-*` form.
- TF tree: `map → odom → base_footprint → base_link → {wheels, lidar_link, camera_link, imu_link}`.
  DiffDrive publishes `odom→base_footprint`; slam_toolbox publishes `map→odom`.
- Topics: `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, `/camera/*`, `/imu` (bridged in `acadbot_gazebo/config/ros_gz_bridge.yaml`).
- Validated working: colcon build clean, robot spawns, `/odom` + `/scan` (≈9 Hz) + TF publish.
- Edit the URDF in `urdf/*.xacro`; re-`colcon build` after changes.

## Slides (course material, kept one level up outside this repo)
The 4 session decks are generated from `python-pptx` scripts styled to the
inmind.ai brand (black cover/section slides, white content slides, blue→purple
gradients). Not part of the code build.
