#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zenoh测试客户端
功能：从points.json获取所有点并循环发送，每个点都等待执行反馈
"""

import zenoh
import json
import time
import sys

# Zenoh 配置
ZENOH_ENDPOINT = "tcp/192.168.20.117:7777"
ROBOT_ID = "robot1"  # 机器人ID，可根据实际情况修改
ACTION_TOPIC = f"edge/{ROBOT_ID}/action"
# ACTION_TOPIC = f"edge/action"
FEEDBACK_TOPIC = f"edge/{ROBOT_ID}/feedback"
POINTS_FILE = "points.json"

# 全局变量用于跟踪反馈状态
feedback_received = False
feedback_data = None
sent_point_id = None

def on_feedback_received(sample):
    """处理接收到的反馈消息"""
    global feedback_received, feedback_data
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
        
        # 检查是否是当前发送点的反馈
        if data.get('point_id') == sent_point_id:
            feedback_received = True
            feedback_data = data
            
            print(f"\n{'='*50}")
            print(f"收到导航反馈")
            print(f"点ID: {data.get('point_id')}")
            print(f"状态: {data.get('status')}")
            print(f"消息: {data.get('message')}")
            print(f"{'='*50}")
            
    except Exception as e:
        print(f"处理接收到的反馈失败: {e}")

def main():
    global sent_point_id, feedback_received, feedback_data
    
    # 读取points.json获取所有点
    try:
        with open(POINTS_FILE, 'r', encoding='utf-8') as f:
            points = json.load(f)
            if not points:
                print("错误: points.json中没有数据")
                return
            
            print(f"成功读取points.json，共获取到 {len(points)} 个点")
    except Exception as e:
        print(f"读取points.json失败: {e}")
        return
    
    # 连接Zenoh
    print(f"连接到Zenoh路由器: {ZENOH_ENDPOINT}")
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ZENOH_ENDPOINT]))
    
    with zenoh.open(config) as session:
        # 订阅反馈主题
        subscription = session.declare_subscriber(
            FEEDBACK_TOPIC,
            on_feedback_received
        )
        print(f"已订阅反馈主题: {FEEDBACK_TOPIC}")
        
        # 循环发送每个点
        for index, point in enumerate(points):
            print(f"\n{'-'*60}")
            print(f"处理第 {index+1}/{len(points)} 个点: {point.get('name', 'Unnamed')}")
            print(f"坐标: x={point['x']}, y={point['y']}")
            
            # 重置反馈状态
            feedback_received = False
            feedback_data = None
            
            # 准备发送数据
            sent_point_id = f"point_{index}_{int(time.time())}"
            action_data = {
                "x": point['x'],
                "y": point['y'],
                "z": point.get('z', 0.0),
                "qx": point.get('qx', 0.0),
                "qy": point.get('qy', 0.0),
                "qz": point.get('qz', 0.0),
                "qw": point.get('qw', 1.0),
                "point_id": sent_point_id
            }
            
            # 发送数据
            action_json = json.dumps(action_data)
            session.put(ACTION_TOPIC, action_json)
            print(f"成功发送点到主题: {ACTION_TOPIC}")
            print(f"发送的数据: {action_json}")
            print("等待导航执行反馈...")
            
            # 等待反馈或超时
            timeout = 300  # 5分钟超时
            start_time = time.time()
            while not feedback_received and (time.time() - start_time) < timeout:
                time.sleep(1)
            
            if feedback_received:
                print("\n导航任务处理完成!")
                if feedback_data.get('status') == 'success':
                    print("[SUCCESS] 导航成功完成")
                else:
                    print("[FAILED] 导航失败")
                
                # 如果不是最后一个点，等待一段时间再发送下一个点
                if index < len(points) - 1:
                    print(f"\n等待 3 秒后发送下一个点...")
                    time.sleep(2)
            else:
                print("\n超时: 未收到导航反馈")
                # 询问是否继续发送下一个点
                if index < len(points) - 1:
                    response = input("是否继续发送下一个点? (y/n): ")
                    if response.lower() != 'y':
                        print("已取消发送剩余点")
                        break
        
        print(f"\n{'-'*60}")
        print("所有点处理完毕!")
                
    

if __name__ == "__main__":
    main()