#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from pupil_apriltags import Detector
import cv2, numpy as np

# 카메라 파라미터
fx, fy, cx, cy = 1018.88, 1016.72, 372.64, 229.31
TAG_SIZE = 0.02  # m

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()

        # AprilTag 감지기 생성
        self.detector = Detector(families="tag36h11")
        self.get_logger().info("AprilTag 감지기 초기화 완료")

        # 이미지 토픽 구독 -> 메시지가 도착하면 self.cb_image()가 호출
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.cb_image, 10)

        # AprilTag의 Pose 정보를 tag_pose 토픽으로 publish  
        self.pub = self.create_publisher(PoseStamped, 'tag_pose', 10)

    # 이미지가 들어오면 실행되는 콜백 함수
    def cb_image(self, msg):
        # Ros 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # cv2.imwrite("/tmp/debug_frame.jpg", frame)

        # detect() 호출 -> AprilTag를 찾고, 태그가 있다면 pose를 추정
        tags = self.detector.detect(gray,
                                    estimate_tag_pose=True,
                                    camera_params=(fx,fy,cx,cy),
                                    tag_size=TAG_SIZE)
        
        if not tags: # 감지된 태그가 없다면 함수 종료
            self.get_logger().info("AprilTag 감지되지 않음")
            return

        # 태그가 여러개 있을 경우, 첫 번째 태그만 사용
        tag = tags[0]

        # 카메라 기준의 위치(단위: m) 추출 -> control 노드에서 좌표를 받고 mm로 변경 필요
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_frame'

        # 위치 (m)
        pose.pose.position.x = float(tag.pose_t[0,0])
        pose.pose.position.y = float(tag.pose_t[1,0])
        pose.pose.position.z = float(tag.pose_t[2,0])

        self.get_logger().info(f" 위치 (camera 기준): x={pose.pose.position.x :.4f} m, y={pose.pose.position.y:.4f} m, z={pose.pose.position.z:.4f} m")

        # 회전 행렬 → quaternion(쿼터니언) 변환
        ## geometry_msgs/Pose는 orientation을 쿼터니언으로 담tf_transformations.quaternion_from_matrix(M)기 때문에 필수임
        R_mat, _ = cv2.Rodrigues(tag.pose_R)
        q = self.to_quaternion(R_mat)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.get_logger().info(
            f"orientation (quaternion): x={q[0]:.3f}, y={q[1]:.3f}, z={q[2]:.3f}, w={q[3]:.3f}"
        )
        self.pub.publish(pose)

    # 쿼터니언 변환 함수
    ## 행렬 R을 4x4 동차행렬로 만든 후 -> tf_transformations.quaternion_from_matrix(M)로 변환
    def to_quaternion(self, R):
        import tf_transformations
        M = np.eye(4)
        M[:3,:3] = R
        return tf_transformations.quaternion_from_matrix(M)

def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
