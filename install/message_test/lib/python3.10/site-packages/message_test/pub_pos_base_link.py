# publish_base_link_pose.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped

class BaseLinkPosePublisher(Node):
    def __init__(self):
        super().__init__('base_link_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10Hz

    def publish_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',          # 目标坐标系（地图）
                'base_link',    # 源坐标系（机器人）
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            self.pose_pub.publish(pose)
            # 可选：打印到终端
            self.get_logger().info(
                f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, "
                f"yaw={self.quat_to_yaw(pose.pose.orientation):.3f}",
                throttle_duration_sec=1.0  # 每秒最多打一次
            )
        except Exception as e:
            self.get_logger().debug(f"TF not ready: {e}")

    def quat_to_yaw(self, q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()
    node = BaseLinkPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
