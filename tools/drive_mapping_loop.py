#!/usr/bin/env python3
"""
drive_mapping_loop.py — drive AcadBot around the whole academy world, once.

Purpose: build the *reference map* without a human on the keyboard. Use it to
produce the instructor's fallback map before Session 2, or to smoke-test that
mapping still works after a change.

    # terminal 1
    ros2 launch acadbot_bringup mapping.launch.py headless:=true rviz:=false
    # terminal 2
    python3 tools/drive_mapping_loop.py
    ros2 run nav2_map_server map_saver_cli \
        -f src/acadbot_navigation/maps/academy_map

It is a plain proportional waypoint follower on /odom -> /cmd_vel: turn to face
the next waypoint, drive to it, repeat. Speeds are deliberately gentle, because
slow and straight is exactly what scan matching wants.

The route hugs the walls at a 0.5 m margin and covers BOTH sides of the centre
divider, passing through the south corridor to get between them.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# The academy world is 8 m x 6 m. A 0.15 m divider sits at x = 0.5 running from
# y = -1.0 up to the north wall, so the only way across is the south corridor.
ROUTE = [
    (-3.4, -2.4), (-3.4, 2.4), (-0.3, 2.4), (-0.3, -2.4),   # left of divider
    (3.4, -2.4), (3.4, 2.4), (1.0, 2.4), (1.0, -2.4),       # right of divider
    (-3.0, -2.4),                                            # home, closes loop
]

V_MAX = 0.22        # m/s   — slow enough for clean scan matching
W_MAX = 0.50        # rad/s
POS_TOL = 0.18      # m
YAW_TOL = 0.12      # rad


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Driver(Node):
    def __init__(self):
        super().__init__("drive_mapping_loop")
        # rclpy declares use_sim_time itself, so set it rather than re-declare.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.05, self.tick)
        self.pose = None
        self.i = 0
        self.done = False

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def tick(self):
        if self.pose is None or self.done:
            return
        x, y, th = self.pose
        gx, gy = ROUTE[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)

        if dist < POS_TOL:
            self.get_logger().info(
                f"waypoint {self.i + 1}/{len(ROUTE)} reached ({gx:+.1f}, {gy:+.1f})")
            self.i += 1
            if self.i >= len(ROUTE):
                self.pub.publish(Twist())
                self.get_logger().info("route complete — save the map now")
                self.done = True
            return

        err = wrap(math.atan2(dy, dx) - th)
        cmd = Twist()
        if abs(err) > YAW_TOL:                      # square up on the target
            cmd.angular.z = max(-W_MAX, min(W_MAX, 1.2 * err))
        else:                                       # then drive, trimming heading
            cmd.linear.x = min(V_MAX, 0.6 * dist)
            cmd.angular.z = max(-W_MAX, min(W_MAX, 0.8 * err))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    n = Driver()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.pub.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
