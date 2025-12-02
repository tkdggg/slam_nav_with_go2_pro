#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self):
        # 等待 action server 启动
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        # 构造目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.position.z = 0.37  # 注意：通常地面机器人 z=0，确认是否需要
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # GoalStatus.SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = Nav2GoalSender()

    if node.send_goal():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted by user.')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()