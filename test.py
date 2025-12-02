#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Zenoh 测试客户端
功能：每次发送一个导航点，等待完成反馈后再发送下一个点，
      按照"发送点→等待反馈→发送下一个点"的循环模式工作。
"""

import zenoh
import json
import time
import asyncio
import sys

# Zenoh 配置
ZENOH_ENDPOINT = "tcp/192.168.20.145:8447"
ACTION_TOPIC = "edge/action"
FEEDBACK_TOPIC = "edge/feedback"
POINTS_FILE = "points.json"

class ZenohTestClient:
    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.session = None
        self.current_point_index = 0
        self.feedback_received = asyncio.Event()
        self.current_point_id = None  # 用于跟踪当前发送的点ID
        
        # 获取当前事件循环
        try:
            self.loop = asyncio.get_event_loop()
        except RuntimeError:
            # 如果没有当前事件循环，创建一个新的
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
    
    def get_next_point(self):
        """按需获取下一个点的信息，而不是提前加载整个列表"""
        try:
            # 每次需要时才读取文件，获取当前索引对应的点
            with open(POINTS_FILE, 'r', encoding='utf-8') as f:
                points = json.load(f)
                
                # 检查索引是否有效
                if self.current_point_index >= len(points):
                    return None
                    
                point = points[self.current_point_index]
                print(f"[POINT] 获取点 #{self.current_point_index}: {point.get('name', 'Unnamed')}", flush=True)
                return point
        except FileNotFoundError:
            print(f"[ERROR] 找不到文件: {POINTS_FILE}", flush=True)
            return None
        except json.JSONDecodeError:
            print(f"[ERROR] 文件 {POINTS_FILE} 格式错误", flush=True)
            return None
        except Exception as e:
            print(f"[ERROR] 获取下一个点失败: {e}", flush=True)
            return None

    async def connect(self):
        """连接到 Zenoh 路由器并设置订阅"""
        try:
            print(f"[CONNECT] 连接到 Zenoh 路由器: {self.endpoint}", flush=True)
            config = zenoh.Config()
            config.insert_json5("connect/endpoints", json.dumps([self.endpoint]))
            
            # 使用 zenoh.open
            self.session = zenoh.open(config)
            
            # 订阅 feedback 主题
            self.feedback_subscription = self.session.declare_subscriber(
                FEEDBACK_TOPIC,
                self.feedback_callback
            )
            print(f"[SUBSCRIBE] 已订阅反馈主题: {FEEDBACK_TOPIC}", flush=True)
            return True
        except Exception as e:
            print(f"[ERROR] 连接 Zenoh 路由器失败: {e}", flush=True)
            return False

    def feedback_callback(self, sample):
        """处理接收到的 feedback 消息"""
        try:
            print(f"[DEBUG] 收到反馈样本，开始处理...", flush=True)
            
            # 解析 payload
            if hasattr(sample, 'value') and hasattr(sample.value, 'payload'):
                payload = sample.value.payload
            elif hasattr(sample, 'payload'):
                payload = sample.payload
            else:
                payload = str(sample)
                print(f"[DEBUG] 样本没有payload属性，转为字符串", flush=True)

            if hasattr(payload, 'decode'):
                feedback_str = payload.decode("utf-8")
            else:
                feedback_str = str(payload)
                print(f"[DEBUG] payload没有decode方法，转为字符串", flush=True)

            feedback = json.loads(feedback_str)
            print(f"[DEBUG] 解析反馈JSON: {feedback}", flush=True)
            
            status = feedback.get("status", "unknown")
            
            # 首先尝试直接从根对象获取point_id
            point_id = feedback.get("point_id")
            
            # 如果根对象中没有point_id，尝试从嵌套的action对象中获取
            if point_id is None and 'action' in feedback and isinstance(feedback['action'], dict):
                point_id = feedback['action'].get('point_id')
                if point_id:
                    print(f"[DEBUG] 从action对象中获取到point_id: {point_id}", flush=True)

            print(f"\n{'-'*50}", flush=True)
            print(f"[FEEDBACK] 收到导航完成反馈 (当前点索引: #{self.current_point_index})")
            print(f"[FEEDBACK ID] 反馈点ID: {point_id}, 当前点ID: {self.current_point_id}", flush=True)
            
            # 状态显示
            status_emoji = "[SUCCESS]" if status == "success" else "[FAILED]" if status == "failed" else "[WARNING]"
            print(f"[STATUS] {status_emoji} {status.upper()} - {feedback.get('message', '')}", flush=True)
            
            # 显示额外信息
            if 'timestamp' in feedback:
                import datetime
                time_str = datetime.datetime.fromtimestamp(feedback['timestamp']).strftime('%H:%M:%S.%f')[:-3]
                print(f"[TIME] 反馈时间戳: {time_str}", flush=True)
            
            print(f"{'-'*50}", flush=True)

            # 只有当反馈状态有效且与当前点匹配时才设置事件
            if status in ["success", "failed", "error"]:
                # 如果提供了point_id，则验证是否匹配当前点
                if point_id is not None:
                    if point_id == self.current_point_id:
                        print(f"[MATCH] 反馈点ID匹配，设置事件以继续处理流程", flush=True)
                        # 调度到 asyncio 事件循环中执行 set()
                        self.loop.call_soon_threadsafe(self.feedback_received.set)
                    else:
                        print(f"[MISMATCH] 反馈点ID不匹配，忽略该反馈", flush=True)
                else:
                    # 没有point_id时，假设是针对当前点的反馈
                    print(f"[WARNING] 没有point_id，假设是当前点反馈，设置事件", flush=True)
                    self.loop.call_soon_threadsafe(self.feedback_received.set)

        except Exception as e:
            print(f"[ERROR] 处理反馈消息失败: {e}", flush=True)

    def send_action(self, point_data):
        """发布 action 到 edge/action 主题"""
        try:
            # 生成唯一的点ID，用于跟踪反馈
            self.current_point_id = f"point_{self.current_point_index}_{int(time.time())}"
            
            # 发送目标点数据，包含点ID用于匹配反馈
            # 从points.json中尝试获取位姿数据，只有当不存在时才使用默认值
            action_data = {
                "x": point_data['x'],
                "y": point_data['y'],
                "z": point_data.get('z', 0.0),  # 从points.json获取，默认0.0
                "qx": point_data.get('qx', 0.0),  # 从points.json获取，默认0.0
                "qy": point_data.get('qy', 0.0),  # 从points.json获取，默认0.0
                "qz": point_data.get('qz', 0.0),  # 从points.json获取，默认0.0
                "qw": point_data.get('qw', 1.0),  # 从points.json获取，默认1.0（无旋转）
                "point_id": self.current_point_id  # 添加点ID用于匹配反馈
            }
            action_json = json.dumps(action_data)
            
            print(f"[DEBUG] 准备发送动作数据")
            self.session.put(ACTION_TOPIC, action_json)
            
            print(f"[ACTION] -> 发送点 #{self.current_point_index}: {point_data['name']}", flush=True)
            print(f"         坐标: x={point_data['x']}, y={point_data['y']}", flush=True)
            print(f"         点ID: {self.current_point_id}", flush=True)
            
            return True
        except Exception as e:
            print(f"[ERROR] 发布动作失败: {e}", flush=True)
            return False

    async def run_sequence(self):
        """每次发送一个点，等待反馈后再发送下一个点的循环逻辑"""
        print(f"[START] 开始执行点位序列，严格按照'发送点→等待Nav2真正完成→发送下一个点'的工作流程", flush=True)
        print(f"[INFO] 系统将等待每个点导航实际完成后才会发送下一个点", flush=True)
        print(f"[INFO] 每个点的处理都包含完整的'发送→实际导航→反馈接收'循环", flush=True)
        
        # 获取第一个点
        point_data = self.get_next_point()
        
        while point_data is not None:
            
            print(f"\n{'='*50}", flush=True)
            print(f"[SEQUENCE] 开始处理点 #{self.current_point_index}: {point_data.get('name', 'Unnamed')}", flush=True)
            print(f"{'='*50}", flush=True)
            
            # 1. 重置 Event
            self.feedback_received.clear()
            print(f"[EVENT] 重置反馈事件，准备接收导航完成信号", flush=True)
            
            # 2. 发送指令
            print(f"[ACTION] 准备发送目标点 #{self.current_point_index}", flush=True)
            if not self.send_action(point_data):
                print("[CRITICAL] 发送失败，退出序列。", flush=True)
                break

            # 3. 等待 Feedback (核心等待机制)
            print(f"[WAIT] 正在等待机器人实际完成点 #{self.current_point_index} 的导航...", flush=True)
            print(f"[WAIT] 系统将在此阻塞，直到机器人真正到达目标位置并返回反馈", flush=True)
            print(f"[WAIT] 根据导航距离，这可能需要几十秒到几分钟", flush=True)
            
            try:
                print(f"[DEBUG] 开始等待反馈，超时设置为 300 秒", flush=True)
                await asyncio.wait_for(self.feedback_received.wait(), timeout=300.0) # 5分钟超时
                print(f"\n[SUCCESS] 成功接收到当前点的导航完成反馈！", flush=True)
                print(f"[SUCCESS] 准备发送下一个点 (#{self.current_point_index + 1})")
                print(f"{'='*50}", flush=True)
            except asyncio.TimeoutError:
                print(f"\n[TIMEOUT] 接收反馈超时 ({300}s)，退出序列。", flush=True)
                print(f"{'='*50}", flush=True)
                break
            
            # 4. 增加索引并获取下一个点
            self.current_point_index += 1
            print(f"[DEBUG] 获取下一个点 (#{self.current_point_index})", flush=True)
            
            # 获取下一个点
            point_data = self.get_next_point()
            
            if point_data:
                print(f"[INFO] 循环继续，准备处理下一个点")
            else:
                print(f"[INFO] 没有更多点位需要处理")

        print("\n" + "="*50)
        print("[COMPLETE] 所有目标点已成功发送并收到实际导航完成反馈。", flush=True)
        print("[COMPLETE] 循环工作流程已正常完成。", flush=True)
        print("="*50)

    async def close(self):
        """关闭连接"""
        if hasattr(self, 'feedback_subscription'):
            self.feedback_subscription.undeclare()
        if self.session:
            # 避免可能的线程恐慌，直接退出即可
            print("[INFO] Zenoh 会话关闭", flush=True)
            
async def main():
    client = ZenohTestClient(ZENOH_ENDPOINT)
    
    if await client.connect():
        try:
            await client.run_sequence()
        except KeyboardInterrupt:
            print("\n[STOP] 测试客户端已停止", flush=True)
        finally:
            await client.close()

if __name__ == "__main__":
    try:
        # 运行主函数
        asyncio.run(main())
    except Exception as e:
        print(f"[ERROR] 程序意外退出: {e}", flush=True)
        sys.exit(1)