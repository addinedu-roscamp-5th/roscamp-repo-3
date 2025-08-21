"""
coordinate_transform.py
카메라 좌표를 로봇 base 좌표계로 변환하는 모듈
"""
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# Hand-Eye Calibration 결과 (카메라 → 그리퍼)
X_matrix = np.array([
    [ 0.7039,  0.7102, -0.0089,  -35.56 ],  # X축 기준
    [-0.7100,  0.7033, -0.0347,  -35.16 ],  # Y축 기준
    [-0.0184,  0.0307,  0.9994,    5.92 ],  # Z축 기준
    [ 0,       0,       0,        1.0   ]
])

# ──────────────────────────────────────────────
# 1. DH 파라미터 정의
# ──────────────────────────────────────────────
def get_robot_dh_params():
    d_vals     = [131.22, 0,     0,    63.4, 75.05, 45.6]
    a_vals     = [0,     -110.4, -96,   0,    0,     0]
    alpha_vals = [1.5708, 0,     0,    1.5708, -1.5708, 0]
    offsets    = [0,     -1.5708, 0,  -1.5708, 1.5708, 0]
    dh_params = []
    for i in range(6):
        dh_params.append([a_vals[i], alpha_vals[i], d_vals[i], offsets[i]])
    return dh_params

# ──────────────────────────────────────────────
# 2. 단일 DH 변환 행렬
# ──────────────────────────────────────────────
def dh_transformation_matrix(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0 ,      sa,       ca,      d],
        [0 ,       0,        0,      1]
    ])

# ──────────────────────────────────────────────
# 3. Forward Kinematics
# ──────────────────────────────────────────────
def forward_kinematics(joint_angles, dh_params):
    T = np.eye(4)
    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        T_i = dh_transformation_matrix(a, alpha, d, theta)
        T = T @ T_i
    return T

# ──────────────────────────────────────────────
# 4. Gripper → Base 변환 행렬
# ──────────────────────────────────────────────
def get_gripper_to_base_matrix(joint_angles):
    dh_params = get_robot_dh_params()
    T_gripper_to_base = forward_kinematics(joint_angles, dh_params)
    return T_gripper_to_base

# ──────────────────────────────────────────────
# 5. 타겟 pose 전체 변환 (회전 포함)
# ──────────────────────────────────────────────
def transform_target_pose_camera_to_base(camera_coords, rvec_deg, joint_angles):
    """
    카메라 기준 타겟 pose → 그리퍼 기준 → 베이스 기준으로 변환
    Args:
        camera_coords: [x, y, z] 카메라 좌표계 위치 (mm)
        rvec_deg: [rx, ry, rz] 회전벡터 (degree)
        joint_angles: 로봇 관절 각도 (라디안)
    Returns:
        [x, y, z, roll, pitch, yaw]: 베이스 기준 위치 및 자세 (mm, degree)
    """
    # 1. 회전벡터 → 회전행렬 (이미지 출력 회전은 디그리라서 라디안으로 변환해줌)
    rvec_rad = np.deg2rad(rvec_deg)
    R_cam, _ = cv2.Rodrigues(rvec_rad)
    t_cam = np.array(camera_coords).reshape(3, 1)
    
    # 2. 카메라 좌표계 pose 행렬 만들기
    T_target_in_camera = np.eye(4)
    T_target_in_camera[:3, :3] = R_cam
    T_target_in_camera[:3, 3] = t_cam.flatten()
    
    # 3. 카메라 → 그리퍼
    T_target_in_gripper = X_matrix @ T_target_in_camera
    
    # 4. 그리퍼 → 베이스
    T_gripper_to_base = get_gripper_to_base_matrix(joint_angles)
    T_target_in_base = T_gripper_to_base @ T_target_in_gripper
    
    # 5. 위치 및 회전 추출
    position = T_target_in_base[:3, 3]  # [x, y, z]
    rotation_matrix = T_target_in_base[:3, :3]
    
    # 회전행렬 → Euler angles (roll, pitch, yaw)
    r = R.from_matrix(rotation_matrix)
    rpy = r.as_euler('xyz', degrees=True)  # deg로 보기 좋게
    
    # xyz + rpy를 한 번에 반환
    combined = position.tolist() + rpy.tolist()
    
    return combined