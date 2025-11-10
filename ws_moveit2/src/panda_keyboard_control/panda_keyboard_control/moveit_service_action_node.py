#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from panda_interfaces.srv import MoveCommand
from panda_interfaces.action import PickPlace
from rclpy.action import ActionServer
import subprocess, time

class MoveItInterface(Node):
    def __init__(self):
        super().__init__('moveit_interface')
        self.srv = self.create_service(MoveCommand, 'moveit_command', self.srv_cb)
        self.action_server = ActionServer(self, PickPlace, 'pick_place', self.execute_pick_place)
        self.proc = None
        self.get_logger().info('MoveIt Interface ready — call /moveit_command or /pick_place')

    def srv_cb(self, request, response):
        cmd = request.command.lower()
        self.get_logger().info(f"Command received: {cmd}")
        if cmd == 'home':
            self.run_demo('mtc_demo.launch.py')
        elif cmd == 'pick':
            self.run_demo('pick_place_demo.launch.py')
        else:
            response.success = False
            response.message = f"Unknown command: {cmd}"
            return response
        response.success = True
        response.message = f"Executed {cmd}"
        return response

    def execute_pick_place(self, goal_handle):
        self.get_logger().info(f"Action goal received for object {goal_handle.request.object_name}")
        goal_handle.publish_feedback(PickPlace.Feedback(progress=0.1))
        self.run_demo('pick_place_demo.launch.py')
        for i in range(10):
            time.sleep(0.5)
            goal_handle.publish_feedback(PickPlace.Feedback(progress=(i+1)/10.0))
        goal_handle.succeed()
        result = PickPlace.Result()
        result.success = True
        result.message = "Pick-place completed"
        return result

    def run_demo(self, launchfile):
        if self.proc and self.proc.poll() is None:
            self.get_logger().info("Demo already running")
            return
        self.proc = subprocess.Popen(["ros2", "launch", "moveit2_tutorials", launchfile])

def main(args=None):
    rclpy.init(args=args)
    node = MoveItInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.proc:
        node.proc.terminate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
