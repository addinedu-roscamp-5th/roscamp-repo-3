#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class ArucoPosesPublisher(Node):
    def __init__(self):
        super().__init__('aruco_poses_publisher')

        # 1) 토픽 퍼블리셔를 저장할 리스트 이름을 publishers → pose_publishers 로 변경
        self.pose_publishers = []
        for i in range(1, 4):
            topic = f'/aruco_pose{i}'
            pub = self.create_publisher(PoseStamped, topic, 10)
            self.pose_publishers.append(pub)
            self.get_logger().info(f'Publisher created for {topic}')

        # 2) 10Hz 타이머
        self.timer = self.create_timer(0.1, self.publish_all_poses)

        # 예시 좌표 & yaw 리스트
        self.positions = [
            (1.0, 1.0, 0.0),
            (2.0, 0.5, 1.57),
            (0.0, 2.0, 3.14),
        ]

    def publish_all_poses(self):
        now = self.get_clock().now().to_msg()
        for idx, pub in enumerate(self.pose_publishers):
            x, y, yaw = self.positions[idx]

            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = 'map'

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, yaw)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            pub.publish(msg)
            self.get_logger().debug(
                f'[{idx+1}] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
