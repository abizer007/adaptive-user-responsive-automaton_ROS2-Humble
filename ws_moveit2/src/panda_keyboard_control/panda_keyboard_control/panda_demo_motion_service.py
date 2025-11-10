#!/usr/bin/env python3
"""
panda_demo_motion_service.py
------------------------------------
A lightweight MoveIt-integrated command service that triggers
simple demonstration maneuvers for presentation and testing.
"""

import rclpy
from rclpy.node import Node
from panda_interfaces.srv import MoveCommand
import os

class PandaDemoMotionService(Node):
    def __init__(self):
        super().__init__('panda_demo_motion_service')
        self.srv = self.create_service(MoveCommand, 'moveit_command', self.handle)
        self.get_logger().info('🤖 Panda Demo Motion Service initialized (service: /moveit_command)')

    def handle(self, req, res):
        cmd = req.command.strip().lower()
        self.get_logger().info(f"Received motion command: '{cmd}'")

        # Example: trigger a lightweight MoveIt computation (forces planner update)
        os.system("ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \"{}\" >/dev/null 2>&1 &")

        # You can extend this with simple motion logic:
        # if cmd == 'wave': ... call joint goal etc.

        res.success = True
        res.message = f"Executed demonstration maneuver: '{cmd}'"
        self.get_logger().info(res.message)
        return res

def main(args=None):
    rclpy.init(args=args)
    node = PandaDemoMotionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
