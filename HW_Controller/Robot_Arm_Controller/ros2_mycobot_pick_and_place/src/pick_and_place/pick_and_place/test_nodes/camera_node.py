#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request

class CameraFromStreamNode(Node):
    def __init__(self):
        super().__init__('camera_node_from_stream')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Flask MJPEG 스트림 주소
        self.stream_url = "http://192.168.5.1:5010/stream"
        self.bytes = b''

        try:
            self.stream = urllib.request.urlopen(self.stream_url)
            self.get_logger().info(f"Stream 연결 성공: {self.stream_url}")
        except Exception as e:
            self.get_logger().error(f"tream 연결 실패: {e}")
            return

        # 주기적 프레임 읽기 (0.3초 마다)
        self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        try:
            self.bytes += self.stream.read(1024)

            # MJPEG 스트림은 JPEG 이미지를 연속적으로 전송
            start = self.bytes.find(b'\xff\xd8')  # JPEG 시작
            end = self.bytes.find(b'\xff\xd9')    # JPEG 끝

            if start != -1 and end != -1:
                jpg = self.bytes[start:end+2]
                self.bytes = self.bytes[end+2:]

                # JPEG 바이트 배열 → OpenCV 이미지로 디코딩
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    # ROS2에서 사용 가능한 sensor_msgs/Image로 변환 후 /camera/image_raw 토픽으로 퍼블리시
                    print(img)
                    # msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                    # msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가 (optional but good)
                    # msg.header.frame_id = 'camera_frame'  # ← 여기를 추가
                    # self.pub.publish(msg)
                else:
                    self.get_logger().warn("JPEG 디코딩 실패")
        except Exception as e:
            self.get_logger().error(f"스트림 오류: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraFromStreamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

