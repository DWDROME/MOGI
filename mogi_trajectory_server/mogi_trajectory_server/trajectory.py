"""
文件名: trajectory_publisher.py
描述: 实现一个 ROS 2 节点，用于基于里程计数据生成并发布机器人轨迹。
    - 订阅里程计数据 (`nav_msgs/Odometry`)。
    - 根据里程计数据生成轨迹并发布 (`nav_msgs/Path`)。
    - 支持自定义发布频率、最小移动距离等参数。
作者: ZiYang Chen
日期: 2025-01-03
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryPublisher(Node):
    """
    实现轨迹发布器节点，用于订阅里程计数据并生成轨迹。
    """
    def __init__(self):
        super().__init__('trajectory_publisher')

        # 声明可配置参数
        self.declare_parameter("trajectory_topic", "trajectory")  # 轨迹话题名称
        self.declare_parameter("odometry_topic", "odom")  # 里程计话题名称
        self.declare_parameter("frame_id", "odom")  # 坐标系 ID
        self.declare_parameter("publish_rate", 2.0)  # 发布频率 (Hz)
        self.declare_parameter("min_distance", 0.1)  # 最小移动距离 (m)

        # 获取参数值
        self.publish_rate = self.get_parameter("publish_rate").value
        self.min_distance = self.get_parameter("min_distance").value

        # 创建发布器和订阅器
        self.path_pub = self.create_publisher(Path, self.get_parameter("trajectory_topic").value, 10)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self.odom_callback, 10)

        # 创建定时器，按固定频率发布轨迹
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_trajectory)

        # 初始化轨迹数据
        self.last_pose = None  # 上一次记录的位姿
        self.path = Path()  # 初始化轨迹路径
        self.path.header.frame_id = self.get_parameter("frame_id").value  # 设置轨迹的坐标系 ID

    def odom_callback(self, msg):
        """
        里程计订阅回调函数，用于处理里程计数据并生成轨迹点。
        """
        # 从里程计数据生成 PoseStamped 消息
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # 如果没有记录过位姿，将当前位姿加入轨迹
        if self.last_pose is None:
            self.path.poses.append(pose)
            self.last_pose = pose
            return

        # 计算机器人在二维平面上的移动距离
        distance_moved = math.sqrt(
            (pose.pose.position.x - self.last_pose.pose.position.x) ** 2 +
            (pose.pose.position.y - self.last_pose.pose.position.y) ** 2
        )

        # 如果移动距离大于最小阈值，将当前位姿加入轨迹
        if distance_moved >= self.min_distance:
            self.path.poses.append(pose)
            self.last_pose = pose

    def publish_trajectory(self):
        """
        按定时器触发，发布生成的轨迹。
        """
        # 更新轨迹的时间戳
        self.path.header.stamp = self.get_clock().now().to_msg()
        # 发布轨迹
        self.path_pub.publish(self.path)

def main(args=None):
    """
    主函数，初始化并运行 TrajectoryPublisher 节点。
    """
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)  # 开始事件循环
    trajectory_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 ROS 2 通信

if __name__ == '__main__':
    main()
