#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import tf_transformations

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')

        # 카메라 → 그리퍼 변환행렬 (Hand-Eye Calibration 결과) / 단위: mm
        self.X_matrix = np.array([
            [0.7039,  0.7102, -0.0089,  -35.56],
            [-0.7100,  0.7033, -0.0347,  -35.16],
            [-0.0184,  0.0307,  0.9994,    5.92],
            [0,       0,       0,        1.0]
        ])

        # 로봇 현재 관절 값 (rad 단위)
        self.joint_angles = [-0.113, -0.006, 0.123, -1.321, -0.051, 0.768]

        # DH 파라미터 정의
        self.dh_params = self.get_robot_dh_params()

        # 구독: 카메라 기준 태그 pose
        self.sub = self.create_subscription(
            PoseStamped, 'tag_pose', self.cb_tag, 10)

        # 발행: base 기준 target pose
        self.pub = self.create_publisher(
            PoseStamped, 'target_base_pose', 10)

    # -------------------------
    # 콜백 함수
    # -------------------------
    def cb_tag(self, msg):
        # 카메라 기준의 태그 위치 좌표 추출 (단위: m -> mm)
        cam = [
                msg.pose.position.x * 1000.0, 
                msg.pose.position.y * 1000.0, 
                msg.pose.position.z * 1000.0
              ]

        # 쿼터니언 -> Euler 변환
        ## 그 결과를 degree로 변환
        rpy = self.quaternion_to_euler(msg.pose.orientation)  # rad
        rpy_deg = np.rad2deg(rpy)

        # 1. 회전 행렬
        rvec_rad = np.deg2rad(rpy_deg)
        R_cam, _ = cv2.Rodrigues(rvec_rad)
        t_cam = np.array(cam).reshape(3, 1)

        # 2. 카메라 기준 Pose 행렬
        T_camera = np.eye(4)
        T_camera[:3, :3] = R_cam
        T_camera[:3, 3] = t_cam.flatten()

        # 3. 그리퍼 기준
        T_gripper = self.X_matrix @ T_camera

        # 4. 베이스 기준
        T_base = self.forward_kinematics(self.joint_angles, self.dh_params) @ T_gripper

        # 5. ROS 메시지 변환
        # base 좌표 기준으로 헤더 설정
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'base_link'

        # mm에서 m로 변환하여 ROS 표준 단위 사용
        pose.pose.position.x = float(T_base[0, 3] / 1000.0)  # mm → m
        pose.pose.position.y = float(T_base[1, 3] / 1000.0)
        pose.pose.position.z = float(T_base[2, 3] / 1000.0)

        # 회전 행렬 -> 쿼터니언 변환 
        ## ROS 메시지는 orientation을 쿼터니언으로 담아야 함
        q = R.from_matrix(T_base[:3, :3]).as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q

        # 최종 PoseStamped 메시지를 /target_base_pose로 publish
        self.pub.publish(pose)

    # -------------------------
    # 유틸 함수들
    # -------------------------
    def quaternion_to_euler(self, q):
        quat = [q.x, q.y, q.z, q.w]
        return tf_transformations.euler_from_quaternion(quat)  # roll, pitch, yaw [rad]

    def get_robot_dh_params(self):
        d_vals     = [131.22, 0,     0,    63.4, 75.05, 45.6]
        a_vals     = [0,     -110.4, -96,   0,    0,     0]
        alpha_vals = [1.5708, 0,     0,    1.5708, -1.5708, 0]
        offsets    = [0,     -1.5708, 0,  -1.5708, 1.5708, 0]
        dh_params = []
        for i in range(6):
            dh_params.append([a_vals[i], alpha_vals[i], d_vals[i], offsets[i]])
        return dh_params

    def dh_transformation_matrix(self, a, alpha, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0 ,      sa,       ca,      d],
            [0 ,       0,        0,      1]
        ])

    def forward_kinematics(self, joint_angles, dh_params):
        T = np.eye(4)
        for i, (a, alpha, d, offset) in enumerate(dh_params):
            theta = joint_angles[i] + offset
            T_i = self.dh_transformation_matrix(a, alpha, d, theta)
            T = T @ T_i
        return T

def main():
    rclpy.init()
    node = TransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()