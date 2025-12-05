#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zenoh接收端与Nav2集成
功能：接收test.py发出的导航点，注入Nav2执行导航，并返回反馈
"""

import json
import zenoh
import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Zenoh 配置
ZENOH_ENDPOINT = "tcp/192.168.20.117:7777"
ROBOT_ID = "robot1"  # 机器人ID，可根据实际情况修改
ACTION_TOPIC = f"edge/{ROBOT_ID}/action"
# ACTION_TOPIC = f"edge/action"
FEEDBACK_TOPIC = f"edge/{ROBOT_ID}/feedback"

class Nav2ActionClient(Node):
    """Nav2导航动作客户端"""
    def __init__(self):
        super().__init__('nav2_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('等待Nav2动作服务器...')
        self.last_feedback_time = 0  # 上次打印反馈的时间戳
        self.current_pose_data = None  # 初始化当前导航点数据
        
    def send_goal(self, pose):
        """发送导航目标到Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2服务器已连接，发送导航目标')
        
        # 发送目标并等待结果
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future
    
    def goal_response_callback(self, future):
        """处理导航目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        
        self.get_logger().info('导航目标已被接受，等待结果...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """处理导航反馈，每5秒打印一次"""
        import time
        current_time = time.time()
        
        # 检查是否需要打印反馈（间隔5秒）
        if current_time - self.last_feedback_time >= 5.0:
            feedback = feedback_msg.feedback
            self.get_logger().info(
                f'导航中: 当前位置 x={feedback.current_pose.pose.position.x:.2f}, '
                f'y={feedback.current_pose.pose.position.y:.2f}'
            )
            self.last_feedback_time = current_time
    
    def get_result_callback(self, future):
        """处理导航结果"""
        try:
            result = future.result().result
            status = future.result().status
            
            if status == 4:  # SUCCEEDED
                self.get_logger().info('导航任务成功完成')
                # 发送成功反馈
                send_feedback(self.current_pose_data, "success", "Navigation completed successfully")
            else:
                error_msg = f'导航任务失败，状态码: {status}'
                self.get_logger().error(error_msg)
                # 发送失败反馈
                send_feedback(self.current_pose_data, "failed", f"Navigation failed with status {status}")
        except Exception as e:
            self.get_logger().error(f'处理导航结果时出错: {e}')
            if self.current_pose_data:
                send_feedback(self.current_pose_data, "failed", f"Error processing navigation result: {str(e)}")

# 全局变量，用于存储当前正在执行的导航点数据
nav_client = None
current_pose_data = None
session = None

def create_pose_from_data(data):
    """从接收到的数据创建PoseStamped对象"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'  # 使用地图坐标系
    pose.header.stamp = nav_client.get_clock().now().to_msg()
    
    # 设置位置
    pose.pose.position.x = data.get('x', 0.0)
    pose.pose.position.y = data.get('y', 0.0)
    pose.pose.position.z = data.get('z', 0.0)
    
    # 设置方向（四元数）
    pose.pose.orientation.x = data.get('qx', 0.0)
    pose.pose.orientation.y = data.get('qy', 0.0)
    pose.pose.orientation.z = data.get('qz', 0.0)
    pose.pose.orientation.w = data.get('qw', 1.0)
    
    return pose

def send_feedback(pose_data, status, message):
    """通过Zenoh发送导航反馈"""
    global session
    try:
        # 确保pose_data不为None
        if pose_data is None:
            print("警告: 尝试发送反馈但pose_data为None")
            return
            
        feedback = {
            'point_id': pose_data.get('point_id', 'unknown'),
            'status': status,  # 'success' 或 'failed'
            'message': message,
            'timestamp': time.time()
        }
        
        feedback_json = json.dumps(feedback)
        session.put(FEEDBACK_TOPIC, feedback_json)
        # 使用更安全的打印方式，避免长消息导致的问题
        print(f"发送反馈: point_id={feedback['point_id']}, status={feedback['status']}")
    except Exception as e:
        print(f"发送反馈失败: {e}")

def on_action_received(sample):
    """处理接收到的action消息"""
    global current_pose_data
    try:
        # 解析payload
        if hasattr(sample, 'value') and hasattr(sample.value, 'payload'):
            payload = sample.value.payload
        elif hasattr(sample, 'payload'):
            payload = sample.payload
        else:
            payload = str(sample)
            
        if hasattr(payload, 'decode'):
            payload_str = payload.decode("utf-8")
        else:
            payload_str = str(payload)
        
        # 解析JSON数据
        data = json.loads(payload_str)
        
        print(f"\n{'='*50}")
        print(f"接收到导航点数据")
        print(f"坐标: x={data.get('x')}, y={data.get('y')}")
        print(f"z={data.get('z')}, qx={data.get('qx')}, qy={data.get('qy')}, qz={data.get('qz')}, qw={data.get('qw')}")
        print(f"点ID: {data.get('point_id')}")
        print(f"{'='*50}")
        
        # 存储当前导航点数据
        current_pose_data = data
        nav_client.current_pose_data = data
        
        # 创建PoseStamped对象
        pose = create_pose_from_data(data)
        
        # 发送导航目标到Nav2
        nav_client.send_goal(pose)
        
    except Exception as e:
        error_msg = f"处理接收到的消息失败: {e}"
        print(error_msg)
        # 发送错误反馈
        if current_pose_data:
            send_feedback(current_pose_data, "failed", error_msg)

def main():
    global nav_client, session
    try:
        # 初始化ROS 2
        rclpy.init()
        print("ROS 2初始化成功")
        
        # 创建Nav2动作客户端
        nav_client = Nav2ActionClient()
        
        # 初始化Zenoh
        print(f"连接到Zenoh路由器: {ZENOH_ENDPOINT}")
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", json.dumps([ZENOH_ENDPOINT]))
        
        with zenoh.open(config) as zenoh_session:
            session = zenoh_session  # 保存会话引用，用于发送反馈
            
            print(f"已订阅主题: {ACTION_TOPIC}")
            print("等待接收导航点数据...")
            
            # 订阅主题
            subscription = session.declare_subscriber(
                ACTION_TOPIC,
                on_action_received
            )
            
            # 运行ROS 2节点
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(nav_client)
            
            # 保持程序运行
            try:
                executor.spin()
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                
    except Exception as e:
        print(f"运行出错: {e}")
        sys.exit(1)
    finally:
        if nav_client:
            nav_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()