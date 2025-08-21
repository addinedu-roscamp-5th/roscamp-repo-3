"""
AprilTags에서 camera_coords와 rvec_deg를 구하는 모듈
"""
import cv2
import numpy as np
from pupil_apriltags import Detector

# === 카메라 내부 파라미터 설정 ===
camera_matrix = np.array([[1018.8890899848071, 0., 372.64373648977255],
                          [0., 1016.7247236426332, 229.30521863962326],
                          [0., 0., 1.]], dtype=np.float32)
dist_coeffs = np.array([-0.4664, 2.0392, 0.00035, -0.00077, -16.977], dtype=np.float64)

tag_size = 0.02  # 단위: meter

# === Detector 객체는 한 번만 생성 ===
_apriltag_detector = Detector(families="tag36h11")


def _detect_april_tag(frame, camera_matrix, target_id=None):
    """
    내부 전용: AprilTag를 감지하여 camera_coords와 rvec_deg를 반환

    Args:
        frame (np.ndarray): BGR 이미지
        camera_matrix (np.ndarray): 3x3 카메라 내부 파라미터
        target_id (int, optional): 찾고자 하는 특정 AprilTag ID

    Returns:
        camera_coords (list): [x, y, z] in mm
        rvec_deg (list): [rx, ry, rz] in degrees
        tag_id (int): AprilTag ID
    """
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = _apriltag_detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(fx, fy, cx, cy),
        tag_size=tag_size
    )

    if not tags:
        return None, None, None

    # 특정 ID를 찾는 경우
    if target_id is not None:
        target_tag = None
        for tag in tags:
            if tag.tag_id == target_id:
                target_tag = tag
                break
        
        if target_tag is None:
            print(f"Warning: AprilTag ID={target_id}를 찾을 수 없어서 첫 번째 태그({tags[0].tag_id})를 사용합니다.")
            tag = tags[0]  # fallback: 첫 번째 태그 사용
        else:
            tag = target_tag
    else:
        tag = tags[0]  # 첫 번째 태그 사용 (기존 방식)

    tvec = tag.pose_t * 1000  # meter → mm
    rvec, _ = cv2.Rodrigues(tag.pose_R)

    camera_coords = tvec.flatten().tolist()
    rvec_deg = np.rad2deg(rvec.flatten()).tolist()
    tag_id = tag.tag_id

    return camera_coords, rvec_deg, tag_id


def detect_target(frame, target_id=None):
    """
    외부에서 호출하는 함수: frame을 받아 AprilTag pose를 추출

    Args:
        frame (np.ndarray): BGR 이미지

    Returns:                                       
        camera_coords (list): [x, y, z] in mm
        rvec_deg (list): [rx, ry, rz] in degrees
    """
    return _detect_april_tag(frame, camera_matrix, target_id)  # 추후 YOLO 등으로 교체 가능