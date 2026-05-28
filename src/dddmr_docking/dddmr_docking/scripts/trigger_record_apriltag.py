#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dddmr_sys_core.action import RecordApriltag

class TriggerDockingClient(Node):

    def __init__(self):
        super().__init__('trigger_docking_client')
        self._action_client = ActionClient(self, RecordApriltag, '/tag_docking')

    def send_goal(self):
        goal_msg = RecordApriltag.Goal()
        goal_msg.start = True

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal to trigger MPC docking...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        rclpy.shutdown()
        #self._get_result_future = goal_handle.get_result_async()
        #self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.succeed}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback')

def main(args=None):
    rclpy.init(args=args)

    action_client = TriggerDockingClient()
    action_client.send_goal()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
