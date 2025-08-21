"""
전체 로봇 실행 테스트 로직
"""
import os
import time
import numpy as np
import cv2
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

from base_coordinate_transform import transform_target_pose_camera_to_base
from image_capture import CameraManager  # ← 클래스 가져오기
from image_detection import detect_target  # detect() 내부에서 _detect_april_tag 호출

def main():
    mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")
    
    # 카메라 초기화 (예외 처리 추가)
    try:
        camera = CameraManager()  # 여러 디바이스 자동 시도
    except RuntimeError as e:
        camera = None  # 카메라 없이도 동작하도록

    message = "pick_from_shelf1"
    if message == "pick_from_shelf1":
        print("3번 선반 위치로 이동")
        mc.send_angles([90.26, 30.93, -0.79, -89.2, -0.26, 42.45], 20)
        time.sleep(3)

        print("그리퍼를 완전히 엽니다.")
        mc.set_gripper_value(100, 50)

        cur_joints_rad = mc.get_radians()
        print("라디안:", cur_joints_rad)

        # 프레임 가져오고, 프레임에서 에이프릴테그 감지
        frame = camera.get_frame()
        camera_coords, rvec_deg, tag_id = detect_target(frame, target_id=4)

        if camera_coords is not None and rvec_deg is not None:
            print("\n=== April Tag 좌표 정보 ===")
            print(f"카메라 기준 좌표: {camera_coords}")
            print(f"회전 벡터 (도): {rvec_deg}")

            print("\n=== Base 좌표계로 변환 중... ===")
            try:
                base_coords = transform_target_pose_camera_to_base(camera_coords, rvec_deg, cur_joints_rad)
                base_coords[3], base_coords[4], base_coords[5] = -177.0, 2.0, -52.0

                print(f"베이스 좌표 [x, y, z, roll, pitch, yaw]: {base_coords}")

                approach = base_coords.copy()
                approach[0] -= 10
                approach[2] += 65
                print(f"1차 Joint 접근 목표: {approach}")

                mc.send_coords(approach, 20, 1)
                print("로봇 이동(2단계) 명령을 전송했습니다.")
                print("이동 완료까지 대기 중...")
                time.sleep(3)

                current_coords = mc.get_coords()
                print(f"이동 후 현재 좌표: {current_coords}")


                time.sleep(3)
                print(f"{tag_id} 에이프릴 테그를 잡기 위해 그리퍼를 완전히 닫습니다.")
                mc.set_gripper_value(0, 50)
                time.sleep(1)

                mc.send_angles([90.26, 30.93, -0.79, -89.2, -0.26, 42.45], 20)
                time.sleep(5)
                mc.send_angles([-15.73, -40.78, -6.59, -0.17, 0.96, 43.33], 20)
                time.sleep(5)
                mc.send_angles([-14.41, -62.13, -0.94, 20.3, 2.54, 42.45], 20)
                time.sleep(3)
                mc.set_gripper_value(100, 50)
                time.sleep(3)
                mc.send_angles([90.26, 30.93, -0.79, -89.2, -0.26, 42.45], 20)

            except Exception as e:
                print(f"좌표 변환 또는 로봇 이동 중 오류 발생: {e}")

        else:
            print("April Tag 좌표를 가져올 수 없습니다.")



if __name__ == '__main__':
    main()
