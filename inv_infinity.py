#!/usr/bin/env python3
"""
figure_eight.py

Each lobe = fixed linear + fixed angular velocity.
For a circle of radius R:  omega = v / R
Right lobe: omega negative (clockwise)
Left  lobe: omega positive (counter-clockwise)

Phase switch at crossing point (cx, cy) after MIN_STEPS.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math

CX, CY    = 5.5, 5.5
R         = 2.0
SPEED     = 1.0
OMEGA     = SPEED / R        # = 0.5 rad/s
ARRIVE    = 0.3              # metres to crossing to trigger switch
MIN_STEPS = 100              # ~5 s at 20 Hz, must complete lobe first


class FigureEight(Node):
    def __init__(self):
        super().__init__('figure_eight')

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_cb, 10)

        self.pose     = None
        self.phase    = 'right'   # start with right lobe
        self.steps    = 0

        # Teleport to crossing point, facing up (+y) — correct entry for right lobe
        cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        cli.wait_for_service(timeout_sec=3.0)
        req = TeleportAbsolute.Request()
        req.x     = CX
        req.y     = CY
        req.theta = math.pi / 2      # face +y
        cli.call_async(req)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('Figure-eight started.')

    def _pose_cb(self, msg):
        self.pose = msg

    def _loop(self):
        if self.pose is None:
            return

        cmd = Twist()
        cmd.linear.x = SPEED

        if self.phase == 'right':
            cmd.angular.z = -OMEGA    # clockwise
        else:
            cmd.angular.z = +OMEGA    # counter-clockwise

        self.pub.publish(cmd)
        self.steps += 1

        # Check if back at crossing point
        dist = math.hypot(self.pose.x - CX, self.pose.y - CY)
        if self.steps > MIN_STEPS and dist < ARRIVE:
            if self.phase == 'right':
                self.phase = 'left'
                self.get_logger().info('→ Left lobe')
            else:
                self.phase = 'right'
                self.get_logger().info('→ Right lobe')
            self.steps = 0


def main(args=None):
    rclpy.init(args=args)
    node = FigureEight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()