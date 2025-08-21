import rclpy
from rclpy.node import Node
import time
from pymycobot.mycobot280 import MyCobot280

from pick_and_place.base_coordinate_transform import transform_target_pose_camera_to_base
from pick_and_place.image_capture import CameraManager  # CameraManager 클래스 가져오기
from pick_and_place.image_detection import detect_target  # detect() 내부에서 _detect_april_tag 호출
from robocallee_fms.srv import RobotArmRequest

## Note: pick, place 각각 따로 모듈화하기(분리시켜 놓기)

class Robot1ControlNode(Node):
    def __init__(self):
        super().__init__('robot1_control_node')
        self.srv = self.create_service(
            RobotArmRequest,
            'arm1_service',
            self.arm1_control_callback
        )
        self.get_logger().info("arm1_control 서비스 서버가 시작되었습니다.")
        
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        print("로봇이 연결되었습니다.")

        # 카메라 초기화 (예외 처리 추가)
        try:
            # self.camera = CameraManager()  # 여러 디바이스 자동 시도

            self.camera = CameraManager(enable_streaming=True, flask_port=5000)  # 스트리밍 활성화
            self.get_logger().info("카메라가 성공적으로 초기화되었습니다.")
        except RuntimeError as e:
            self.get_logger().error(f"카메라 초기화 실패: {e}")
            self.camera = None  # 카메라 없이도 동작하도록
 
    def arm1_control_callback(self, request, response):
        pinky_id = request.amr_id
        action = request.action.lower()
        shelf_num = request.shelf_num

        self.get_logger().info(f'[서비스 요청] action: {action}, shelf_num: {shelf_num}, pinky_num: {pinky_id}')

        if action == 'shelf_to_buffer':
            success, msg = self.handle_shelf_to_buffer(pinky_id, shelf_num)
        elif action == 'buffer_to_shelf':
            success, msg = self.handle_buffer_to_shelf(pinky_id, shelf_num)
        else:
            success = False, f"지원하지 않는 action: {action}"

        ## Note: OCR 인식 후 얻은 데이터 저장
        response.robot_id = 1
        response.amr_id = pinky_id
        response.action = msg
        response.model = 'None'
        response.size = -1
        response.color = 'None'
        response.success = success
        return response


##=========================================================================================
    '''
    RobotArm1이 shelf에서 buffer로 물건을 옮기는 함수
    '''
    def handle_shelf_to_buffer(self, pinky_id, shelf_num):
        # TODO: 실제 로봇 로직 작성
        print(f"{shelf_num}번 선반을 위한 초기자세로 이동..")
        if shelf_num == 1 or shelf_num == 2:
            self.mc.send_angles([-19.07, 57.04, -13.18, -36.82, 16.52, 47.37], 25) # 1,2번 선반 보는 초기 자세
        elif shelf_num == 3 or shelf_num == 4:
            self.mc.send_angles([-10.28, 84.11, -119.09, 13.53, 9.22, 46.66], 25)  # 3,4번 선반 보는 초기 자세
        # elif shelf_num == 5 or shelf_num == 6:
        #     self.mc.send_angles([-10.28, 84.11, -119.09, 13.53, 9.22, 46.66],20)   # 4,5번 선반 보는 초기 자세 --> 기획상으로만 존재
        else:
            print("정의되지 않은 선반 번호")

        print("그리퍼를 완전히 엽니다.")
        self.mc.set_gripper_value(100, 50)

        # 프레임 가져오고, 프레임에서 에이프릴테그 감지
        time.sleep(5)
        frame = self.camera.get_frame()
        camera_coords, rvec_deg, tag_id = detect_target(frame, target_id=shelf_num) # 타겟 id = shelf_num

        if camera_coords is not None and rvec_deg is not None:
            print("\n=== April Tag 좌표 정보 ===")
            print(f"카메라 기준 좌표: {camera_coords}")
            print(f"회전 벡터 (도): {rvec_deg}")

            try:
                print("\n=== Base 좌표계로 변환 중... ===")
                base_coords = transform_target_pose_camera_to_base(
                    camera_coords, rvec_deg, self.mc.get_radians()
                )

                base_coords[3], base_coords[4], base_coords[5] = -96, 46.39, -91.65
                print(f"베이스 좌표 [x, y, z, roll, pitch, yaw]: {base_coords}")

                approach_coords = base_coords.copy()

                # 선반의 offset값 설정
                if shelf_num == 1 or shelf_num == 2:
                    approach_coords[0] -= 80
                    approach_coords[1] -= 5
                    approach_coords[2] -= 13 # 2는 8
                elif shelf_num == 3 or shelf_num == 4:
                    approach_coords[0] -= 90
                    approach_coords[1] += 0
                    approach_coords[2] -= 8

                print(f"Joint 접근 목표: {approach_coords}")
                self.mc.send_coords(approach_coords, 25, 0)
                print("접근 위치로 이동 중...")
                time.sleep(2)
                print(f"이동 후 현재 좌표: {self.mc.get_coords()}")

                # 집기
                print("그리퍼를 완전히 닫습니다.")
                self.mc.set_gripper_value(0, 50)
                time.sleep(1)

                #경유지 설정(타겟 충돌 방지)
                print(f"{shelf_num}번 선반 기준 경유지 이동..")
                if shelf_num == 1 or shelf_num == 2:
                    self.mc.send_angles([1.05, 34.71, -81.47, 45.52, -5.62, 44.82], 25)  # 1,2번 선반 경유지
                    time.sleep(2)
                elif shelf_num == 3 or shelf_num == 4:
                    self.mc.send_angles([16.43, -5.53, -109.33, 110.91, -16.61, 51.06], 25) # 3,4번 선반 경유지
                    time.sleep(2)

                    # 돌아오면서 선반에 부딪힘을 방지하기 위해 초기 위치로 경유지를 설정
                    self.mc.send_angles([1.05, 34.71, -81.47, 45.52, -5.62, 44.82], 25)  # 1,2번 선반 경유지
                    time.sleep(2)
                else:
                    print("정의되지 않은 선반 번호")

                # 버퍼별 경유지로 이동 -> 버퍼로 이동 -> 후퇴 경유지 이동
                if pinky_id == 1:
                    # 경유지 이동
                    self.mc.send_angles([-96.06, -43.41, -13.27, -23.81, -5.71, 44.56], 25)
                    time.sleep(2)
                    
                    # 버퍼로 이동  
                    self.mc.send_angles([-99.14, -77.6, 24.78, -23.46, -5.09, 40.51], 25)
                    time.sleep(2)
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)
                
                    # 후퇴 경유지 이동
                    self.mc.send_angles([-96.06, -43.41, -13.27, -23.81, -5.71, 44.56], 25)
                    time.sleep(2)
                elif pinky_id == 2:
                    # 경유지 이동
                    self.mc.send_angles([-83.23, -52.99, 7.11, -37.08, -5.18, 49.83], 25)
                    time.sleep(2)

                    # 버퍼로 이동
                    self.mc.send_angles([-86.22, -62.84, 6.5, -24.96, -2.46, 48.69], 25)
                    time.sleep(2)
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)

                    # 후퇴 경유지 이동
                    self.mc.send_angles([-83.23, -52.99, 7.11, -37.08, -5.18, 49.83], 25)
                    time.sleep(2)

                elif pinky_id == 3:
                    # 경유지 이동
                    self.mc.send_angles([-69.52, -38.75, -13.27, -26.19, -5.44, 65.91], 25)
                    time.sleep(2)

                    # 버퍼로 이동
                    self.mc.send_angles([-66.35, -53.7, -15.9, -14.67, -6.24, 66.88], 25)
                    time.sleep(2)
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)
                    
                    # 후퇴 경유지 이동
                    self.mc.send_angles([-69.52, -38.75, -13.27, -26.19, -5.44, 65.91], 25)
                    time.sleep(2)
                else:
                    print("잘못된 핑키 번호입니다.")
                
                # 놓기
                print("그리퍼를 완전히 엽니다.")
                self.mc.set_gripper_value(100, 50)
                time.sleep(1)

                # 복귀
                print("초기 위치로 복귀합니다.")
                self.mc.send_angles([-19.07, 57.04, -13.18, -36.82, 16.52, 47.37], 25)

            except Exception as e:
                print(f"좌표 변환 또는 이동 중 오류 발생: {e}")
                return False, "좌표 변환 또는 로봇 이동 중 오류 발생"
        else:
            print("April Tag 좌표를 가져올 수 없습니다.")
            return False, '에이프릴 테그 인식 실패' # "선반에서 버퍼로 이동 실패"

        self.get_logger().info(f"{shelf_num}번 선반 → {pinky_id}번 버퍼")
        return True, 'shelf_to_buffer' # "선반에서 버퍼로 이동 완료"
    
##=========================================================================================
    '''
    RobotArm1이 buffer에서 shelf로 물건을 옮기는 함수
    '''
    def handle_buffer_to_shelf(self, pinky_id, shelf_num):
        # TODO: 실제 로봇 로직 작성

        print("\ncamera: 콜렉션 시야 확보 위치로 이동")
        self.mc.send_angles( [-61.34, -90.57, 87.89, -62.92, 4.83, 45.0] , 28)
        time.sleep(1)

        print("그리퍼 열기")
        self.mc.set_gripper_value(100, 50)
        time.sleep(1)
    
     # 프레임 가져오고, 프레임에서 에이프릴테그 감지
        time.sleep(5)
        frame = self.camera.get_frame()
        camera_coords, rvec_deg, tag_id = detect_target(frame, target_id=shelf_num) # 타겟 id = shelf_num

        if camera_coords is not None and rvec_deg is not None:
            print("\n=== April Tag 좌표 정보 ===")
            print(f"카메라 기준 좌표: {camera_coords}")
            print(f"회전 벡터 (도): {rvec_deg}")

            try:
                print("\n=== Base 좌표계로 변환 중... ===")
                base_coords = transform_target_pose_camera_to_base(
                    camera_coords, rvec_deg, self.mc.get_radians()
                )

                # RPY 고정
                base_coords[3], base_coords[4], base_coords[5] = -166.52, 10.74, 132.51

                # XYZ 보정
                base_coords[0] += 10
                base_coords[1] += 30
                base_coords[2] += 65

                print(f"이동 좌표(base): {base_coords}")
                self.mc.send_coords(base_coords, 25, 0)
                time.sleep(1.5)

                print("그리퍼 닫기")
                self.mc.set_gripper_value(0, 50)
                time.sleep(1.5)

                # 위로 살짝 들기
                self.mc.send_angles([-47.63, -32.34, 9.93, -44.47, -11.68, 79.01], 20)  
                time.sleep(1.5)

                if shelf_num == 1:
                    # 3층 안전지대 경유지
                    self.mc.send_angles([1.84, 18.36, -20.3, -8.87, -0.52, 50.18], 20)
                    time.sleep(2)

                    # 물건 플레이스
                    self.mc.send_angles([32.95, -12.3, -46.31, 56.6, -34.54, 47.37], 20) 
                    time.sleep(2)

                    print("물건 내려놓기")
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)

                    # 물건 놓고 후진
                    self.mc.send_angles([32.08, 9.49, -46.84, 34.36, -29.09, 49.21], 20) 
                    time.sleep(2)

                elif shelf_num == 2:
                    # 3층 안전지대 경유지
                    self.mc.send_angles([1.84, 18.36, -20.3, -8.87, -0.52, 50.18], 20)
                    time.sleep(2)

                    # 물건 플레이스
                    self.mc.send_angles([-11.33, -35.68, -1.66, 34.18, 11.68, 47.28], 20) 
                    time.sleep(2)

                    print("물건 내려놓기")
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)

                    # 물건 놓고 후진
                    self.mc.send_angles([-10.01, -3.42, -2.9, -2.19, 1.84, 50.88], 20) 
                    time.sleep(2)

                ## Note: 3,4 경유지 보정 필요
                elif shelf_num == 3:
                    self.mc.send_angles([-5.0, 0.35, -72.94, 46.49, 4.21, 43.33] ,20)
                    time.sleep(2)

                    #2층을 바라보는 각도 
                    self.mc.send_angles([-10.28, 84.11, -119.09, 13.53, 9.22, 46.66],20)
                    time.sleep(2)

                    #경유지
                    self.mc.send_angles([12.3, 31.81, -106.87, 39.46, -6.06, 48.69],20)
                    time.sleep(2)

                    self.mc.send_angles([32.16, -131.92, 94.13, 37.52, -35.33, 49.04], 20) # 물건 플레이스
                    time.sleep(3)

                    print("물건 내려놓기")
                    self.mc.set_gripper_value(100, 50)
                    time.sleep(1)
                    
                elif shelf_num == 4:
                    self.mc.send_angles([]) #4번 경유지
                    time.sleep(2)
                    self.mc.send_angles([-19.07, 57.04, -13.18, -36.82, 16.52, 47.37], 20)
                else:
                    print("정의되지 않은 선반 번호")
                    
                self.mc.send_angles([-65.21, 35.15, -41.92, -46.4, 3.42, 32.78], 20)

            except Exception as e:
                print(f"좌표 변환 또는 이동 중 오류 발생: {e}")
                return False, "좌표 변환 또는 로봇 이동 중 오류 발생"
        else:
            print("April Tag 좌표를 가져올 수 없습니다.")
            return False, '에이프릴 테그 인식 실패' # "선반에서 버퍼로 이동 실패"
        
        self.get_logger().info(f"collection 버퍼 → {shelf_num}번 선반")
        return True, 'buffer_to_shelf' # "버퍼에서 선반으로 이동 완료"
##=========================================================================================


def destroy_node(self):
    """노드 종료 시 리소스 정리"""
    if self.camera:
        self.camera.release()  # 카메라 해제
    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Robot1ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.camera.release()  # 안전하게 종료 시 카메라 해제
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()