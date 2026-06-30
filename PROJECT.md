# AcadBot — Project Design & Session Plan

This document is the instructor-facing design of the SLAM-track project: what the
robot is, how the pieces fit, and what each of the four sessions delivers. The
student-facing quick start is in [`README.md`](README.md).

---

## 1. The story

We use **one robot, one world, four sessions** so concepts compound instead of
resetting each week. Students feel the *problem* before they meet the *tool*:

1. **Session 1 — Intro / Drift.** They drive AcadBot open-loop in a square and
   watch its odometry estimate drift away from ground truth. Pain established.
2. **Session 2 — LiDAR SLAM.** `slam_toolbox` builds a consistent map and the
   `map → odom` correction cancels the drift. Pain solved.
3. **Session 3 — Visual SLAM.** Same robot, the camera now. Features, visual
   odometry, depth — how cameras can do what the LiDAR did, and their trade-offs.
4. **Session 4 — Autonomy.** Nav2 plans and drives on the map from Session 2,
   recovers when stuck, and a **C++ mission node** commands the whole thing.
   This is the final evaluation platform.

## 2. The robot

| Property | Value |
|---|---|
| Drive | Differential (2 driven wheels + caster) |
| Footprint | ~0.30 m × 0.22 m, wheel radius 0.05 m |
| Max speed | 0.26 m/s linear, 1.0 rad/s angular |
| LiDAR | 360-beam 2D, 12 m range, 10 Hz → `/scan` |
| Camera | RGBD 640×480, 15 Hz → `/camera/*` |
| IMU | 100 Hz → `/imu` |

Defined entirely in `acadbot_description` as xacro. The Gazebo `DiffDrive`
plugin consumes `/cmd_vel`, drives the wheels, and publishes wheel odometry on
`/odom` plus the `odom → base_footprint` transform.

## 3. TF tree

```
map ──(slam_toolbox / amcl)──► odom ──(diff_drive)──► base_footprint
                                                         └─► base_link
                                                              ├─► left_wheel / right_wheel
                                                              ├─► caster_wheel
                                                              ├─► lidar_link
                                                              ├─► camera_link ─► camera_optical_link
                                                              └─► imu_link
```

- `odom → base_footprint`  : smooth, high-rate, **drifts** (from the diff-drive).
- `map → odom`             : occasional **correction** (from SLAM / AMCL).
- everything below `base_link` : static, from `robot_state_publisher`.

## 4. Topic graph (who talks to whom)

```
 Gazebo ──/scan──────────────► slam_toolbox ──/map──────────► Nav2 global_costmap
 Gazebo ──/odom──────────────► Nav2 controller / costmaps
 Gazebo ──/clock─────────────► (everything, use_sim_time=true)
 Gazebo ──/camera/*──────────► (Session 3: visual SLAM / inspection)

 patrol_commander (C++) ──navigate_to_pose action──► Nav2 bt_navigator
 Nav2 ──/cmd_vel─────────────► Gazebo diff_drive
 Nav2 behavior_server ──(spin / backup / wait)──► /cmd_vel   (recovery)
```

## 5. Recovery behaviors (Session 4 focus)

Configured in `acadbot_navigation/config/nav2_params.yaml` under
`behavior_server`. When the planner or controller reports failure, the default
Nav2 behavior tree escalates through:

1. **Clear costmap** — drop stale obstacles that may be phantom.
2. **Spin** — rotate in place to re-observe surroundings.
3. **Back up** — reverse a short distance from a blockage.
4. **Wait** — pause for dynamic obstacles / costmap refresh.

A `collision_monitor` adds a last-resort safety stop in front of obstacles.
Students trigger recoveries live by placing an obstacle in the robot's path.

## 6. Session-by-session outcomes

| Session | Date | Theme | Hands-on deliverable | Key packages |
|---|---|---|---|---|
| 1 | 06 Aug | Intro to SLAM, localization, frames, odometry & drift | Drive a square open-loop, measure the drift in RViz | description, gazebo, control (`square_driver`) |
| 2 | 10 Aug | LiDAR SLAM with slam_toolbox, scan matching, loop closure | Build & save a map of `academy_world` | slam, gazebo |
| 3 | 13 Aug | Visual SLAM: features, visual odometry, depth | Inspect the camera/depth/point-cloud streams; VO concepts | gazebo (camera), description |
| 4 | 17 Aug | Integration: Nav2, costmaps, recovery, mission control | Autonomous patrol with a C++ Nav2 client + recoveries | navigation, control (`patrol_commander`), bringup |

Final evaluation (Session 16, 20 Aug) runs on this same stack: localize on a
saved map, navigate to graded waypoints, recover from an introduced blockage.

## 7. Stretch goals / assignment ideas

- Tune the DWB critics so the robot hugs corridors less aggressively.
- Add a second room to `academy_world.sdf` and re-map it.
- Replace the open-loop `square_driver` with a closed-loop one that uses `/odom`.
- Add a custom recovery behavior plugin.
- Swap slam_toolbox localization for AMCL and compare pose stability
  (`navigation.launch.py localization:=amcl`).
