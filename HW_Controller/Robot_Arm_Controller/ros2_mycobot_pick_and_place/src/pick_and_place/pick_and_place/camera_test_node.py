import rclpy
from rclpy.node import Node
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pick_and_place.image_capture import CameraManager
from pick_and_place.image_detection import detect_target  # (카메라 좌표, 회전 벡터 [deg]) 리턴

class AprilTagDetectionNode(Node):
    def __init__(self):
        super().__init__('april_tag_detection_node')
        self.bridge = CvBridge()
        self.camera = CameraManager()
        self.get_logger().info("📸 카메라 초기화 완료")

        # 1초마다 감지 시도
        self.timer = self.create_timer(1.0, self.detect_april_tag)

    def detect_april_tag(self):
        try:
            frame = self.camera.get_frame()
            camera_coords, rvec_deg = detect_target(frame)

            if camera_coords is not None and rvec_deg is not None:
                print("\n🎯 [AprilTag 감지 성공]")
                print(f"📍 카메라 기준 좌표: {camera_coords}")
                print(f"🌀 회전 벡터 (도): {rvec_deg}")
            else:
                print("⚠️ AprilTag 미감지")
        except Exception as e:
            self.get_logger().error(f"❌ 예외 발생: {e}")

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
