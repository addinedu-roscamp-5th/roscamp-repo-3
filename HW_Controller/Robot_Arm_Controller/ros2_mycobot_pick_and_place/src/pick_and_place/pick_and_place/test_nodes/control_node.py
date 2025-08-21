#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymycobot.mycobot280 import MyCobot280
from scipy.spatial.transform import Rotation as R

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # MyCobot USB 포트를 통해 통신 시작
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.set_gripper_value(100, 50)

        # transfrom_node에서 퍼빌리시한 PoseStamped 메시지 수신 -> 콜백 함수 cb_move() 실행
        self.sub = self.create_subscription(
            PoseStamped, 'target_base_pose', self.cb_move, 10)

    # 콜백 함수
    def cb_move(self, msg):
        # 단위 m -> mm로 변경
        x = msg.pose.position.x * 1000 
        y = msg.pose.position.y * 1000
        z = msg.pose.position.z * 1000 + 30

        # 쿼터니언 -> 오일러 변환
        quat = msg.pose.orientation
        rx, ry, rz = R.from_quat([quat.x,quat.y,quat.z,quat.w]).as_euler('xyz', degrees=True)

        # 좌표로 이동
        coords = [x,y,z,rx,ry,rz]
        self.get_logger().info(f'이동: {coords}')
        self.mc.send_coords(coords, 20)
        self.mc.set_gripper_value(0, 50)   # pick
        # place 예시
        place = [200,0,150,180,0,90] # 임시 좌표
        self.mc.send_coords(place, 20)
        self.mc.set_gripper_value(100,50) # place

        # 초기 위치로 이동
        self.mc.send_angles([0, 0, 0, 0, 0, 40], 20)


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
