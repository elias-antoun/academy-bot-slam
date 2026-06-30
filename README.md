# AcadBot — Robotics Academy SLAM Track (Summer 2026)

The hands-on code project for the four **SLAM sessions** (Sessions 12–15) of the
Robotics track. Everything runs in **ROS 2 Jazzy + Gazebo Harmonic** inside a
single Docker image, so every student has an identical environment.

**AcadBot** is a small differential-drive robot with a 2D LiDAR, an RGBD camera
and an IMU. Across the four sessions students take it from "raw odometry that
drifts" all the way to "drives itself around a mapped building, recovering when
it gets stuck."

```
   Session 1  ──►  Session 2  ──►  Session 3  ──►  Session 4
  Intro/Drift    LiDAR SLAM      Visual SLAM     Autonomy
   (odometry      (slam_toolbox   (camera, VO,    (Nav2 + recovery
    drift demo)    builds a map)   depth)          + C++ mission node)
```

---

## 1. Build the Docker image (once)

```bash
cd code/docker
./build_ros_image.sh          # builds  acadbot:jazzy
```

This installs ROS 2 Jazzy, Gazebo Harmonic, Nav2, slam_toolbox and the dev
tooling (see `docker/user_install.sh`).

## 2. Start a container

```bash
cd code/docker
./run_ros_container.sh        # opens a shell, mounts ../ros2_ws at /ros2_ws
```

The `ros2_ws/` source tree is **bind-mounted**, so you edit code on your host
and build inside the container. Open extra shells into the same container by
re-running `./run_ros_container.sh`.

## 3. Build the workspace (inside the container)

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash      # (the container's .bashrc does this for you)
```

---

## What to run, per session

### Session 1 — Odometry & drift
```bash
ros2 launch acadbot_gazebo simulation.launch.py        # robot in Gazebo
ros2 launch acadbot_control square_driver.launch.py    # open-loop C++ driver
rviz2 -d src/acadbot_description/rviz/slam.rviz         # Fixed Frame = odom
```
Drive the open-loop square and watch the estimated pose drift from the true pose.

### Session 2 — LiDAR SLAM with slam_toolbox
```bash
ros2 launch acadbot_bringup mapping.launch.py          # sim + slam_toolbox + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive to build the map
```
When the map looks good, save it (see `acadbot_navigation/maps/README.md`).

### Session 3 — Visual SLAM
The robot already publishes `/camera/image`, `/camera/depth_image` and
`/camera/points`. This session is mostly theory + exploring the camera stream
and visual-odometry concepts on top of the same robot.

### Session 4 — Full autonomy (navigation + recovery + C++ control)
```bash
ros2 launch acadbot_bringup autonomy.launch.py         # sim + Nav2 + localization + RViz
ros2 launch acadbot_control patrol.launch.py           # C++ Nav2 action client
```
The C++ `patrol_commander` sends waypoint goals to Nav2; when the robot is
blocked, Nav2's **recovery behaviors** (clear costmap → spin → back up → wait)
kick in. Block its path with a chair in RViz's view to trigger them live.

---

## Packages

| Package | Role |
|---|---|
| `acadbot_description` | URDF/xacro robot model + sensors, RViz configs |
| `acadbot_gazebo`      | Gazebo Harmonic world, robot spawn, `ros_gz` bridge |
| `acadbot_control`     | **C++** nodes: `square_driver` (drift demo), `patrol_commander` (Nav2 client) |
| `acadbot_slam`        | `slam_toolbox` mapping + localization configs/launch |
| `acadbot_navigation`  | Nav2 params (incl. recovery behaviors), maps, launch |
| `acadbot_bringup`     | One-command launch files per session |

See [`PROJECT.md`](PROJECT.md) for the full architecture, the TF tree, the topic
graph and the session-by-session learning outcomes.
