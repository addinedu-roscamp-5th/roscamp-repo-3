import rclpy
from rclpy.node import Node
import time
from pymycobot.mycobot280 import MyCobot280

from pick_and_place.base_coordinate_transform import transform_target_pose_camera_to_base
from pick_and_place.image_capture import CameraManager  # CameraManager 클래스 가져오기
from pick_and_place.image_detection import detect_target  # detect() 내부에서 _detect_april_tag 호출
from robocallee_fms.srv import RobotArmRequest
from pick_and_place.django_client import ask_django_ocr

## Note: pick, place 각각 따로 모듈화하기(분리시켜 놓기)

class Robot2ControlNode(Node):
    def __init__(self):
        super().__init__('robot2_control_node')
        self.srv = self.create_service(
            RobotArmRequest,
            'arm2_service',
            self.arm2_control_callback
        )
        self.get_logger().info("arm2_control 서비스 서버가 시작되었습니다.")
        
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
 
    def arm2_control_callback(self, request, response):
        
        # 장고에 OCR 결과 받기-----------------------------
        time.sleep(1)

        # 기본값 설정
        shoe_info = {'model': 'None', 'color': 'None', 'size': -1}

        try:
            django_url = 'http://192.168.5.17:8000/gwanje/ocr_from_flask_stream/'
            result = ask_django_ocr(django_url, 'get_shoe_info')

            # None 이 아닐 때만 반영
            if result is not None:
                shoe_info = result
            else:
                self.get_logger().warn("OCR 서버 응답이 None, 기본값 사용")
        except Exception as e:
            self.get_logger().error(f"OCR 서버 요청 실패, 기본값 사용: {e}")

        # 응답 세팅
        response.model = shoe_info.get('model', 'None')
        response.color = shoe_info.get('color', 'None')
        response.size = shoe_info.get('size', -1)

        self.get_logger().info(
            f'[서비스 요청] model: {response.model}, color: {response.color}, size: {response.size}'
        )
        # -----------------------------------------------

        pinky_id = request.amr_id
        action = request.action.lower()
        shelf_num = request.shelf_num

        self.get_logger().info(f'[서비스 요청] action: {action}, shelf_num: {shelf_num}, pinky_num: {pinky_id}')

        if action == 'buffer_to_pinky':
            success, msg = self.handle_buffer_to_pinky(pinky_id)
        elif action == 'pinky_to_buffer':
            success, msg = self.handle_pinky_to_buffer(pinky_id)
        else:
            success, msg = False, f"지원하지 않는 action: {action}"

        ## Note: OCR 인식 후 얻은 데이터 저장
        response.robot_id = 2
        response.amr_id = pinky_id
        response.action = msg
        # response.model = 'None'
        # response.size = -1
        # response.color = 'None'
        response.success = success

        return response


##=========================================================================================
    '''
    RobotArm2가 buffer에서 pinky로 물건을 옮기는 함수
    '''
    def handle_buffer_to_pinky(self, pinky_id):
        # TODO: 실제 로봇 로직 작성
        print(f"{pinky_id} 버퍼를 위한 초기자세로 이동..")
        if pinky_id == 1:
            self.mc.send_angles([137.1, -9.84, -31.28, -30.84, -3.69, 91.14], 25) # 1번 버퍼 보는 초기 자세
        elif pinky_id == 2:
            self.mc.send_angles([119.0, -12.04, -32.34, -36.12, -2.1, 69.78], 25) # 2번 버퍼 보는 초기 자세
        elif pinky_id == 3:
            self.mc.send_angles([86.39, -7.2, -32.34, -39.99, -0.7, 42.8], 25) # 3번 버퍼 보는 초기 자세
        else:
            print("정의되지 않은 핑키 번호")

        print("그리퍼를 완전히 엽니다.")
        self.mc.set_gripper_value(100, 50)
        
        # 프레임 가져오고, 프레임에서 에이프릴테그 감지
        time.sleep(4)
        frame = self.camera.get_frame()
        camera_coords, rvec_deg, tag_id = detect_target(frame, target_id=pinky_id) # 타겟 id = pinky_id

        if camera_coords is not None and rvec_deg is not None:
            print("\n=== April Tag 좌표 정보 ===")
            print(f"카메라 기준 좌표: {camera_coords}")
            print(f"회전 벡터 (도): {rvec_deg}")

            print("\n=== Base 좌표계로 변환 중... ===")
            try:
                base_coords = transform_target_pose_camera_to_base(
                    camera_coords, rvec_deg, self.mc.get_radians()
                )

                # roll, pitch, yaw 고정
                base_coords[3], base_coords[4], base_coords[5] = -177.0, 2.0, -52.0
                print(f"베이스 좌표 [x, y, z, roll, pitch, yaw]: {base_coords}")

                approach_coords = base_coords.copy()
                # 버퍼마다 각각의 offset값 설정
                if pinky_id == 1:
                    approach_coords[0] -= 10
                    approach_coords[1] -= 7
                    approach_coords[2] += 70
                elif pinky_id == 2:
                    approach_coords[0] -= 5
                    approach_coords[1] -= 10
                    approach_coords[2] += 70
                elif pinky_id == 3:
                    approach_coords[0] -= 10
                    approach_coords[1] -= 10
                    approach_coords[2] += 70
                else:
                    print("제대로된 핑키 번호 입력하시오.")

                self.mc.send_coords(approach_coords, 20, 1)
                print("로봇 이동(2단계) 명령을 전송했습니다.")
                print("이동 완료까지 대기 중...")
                time.sleep(1)

                current_coords = self.mc.get_coords()
                print(f"이동 후 현재 좌표: {current_coords}")

                # 물체 잡기
                self.mc.set_gripper_value(0, 50)  # 그리퍼 닫기
                time.sleep(0.5)

                print(f"{pinky_id} 경유 지점으로 이동...")
                if pinky_id == 1:
                    self.mc.send_angles([131.48, -14.23, -49.74, -11.33, 0.35, 93.16], 25) # 1번 버퍼 pick 후 경유 위치로 이동
                elif pinky_id == 2:
                    self.mc.send_angles([115.31, 5.27, -63.54, -7.64, 0.26, 61.61], 25)    # 2번 버퍼 pick 후 경유 위치로 이동
                elif pinky_id == 3:
                    self.mc.send_angles([95.88, 9.58, -56.33, -15.46, -1.14, 55.98], 25)   # 3번 버퍼 pick 후 경유 위치로 이동
                else:
                    print("정의되지 않은 핑키 번호")
                time.sleep(0.5)

            except Exception as e:
                print(f"좌표 변환 또는 로봇 이동 중 오류 발생: {e}")
                return False, "좌표 변환 또는 로봇 이동 중 오류 발생"

        else:
            print("April Tag 좌표를 가져올 수 없습니다.")
            return False, '에이프릴 테그 인식 실패' # "버퍼에서 핑키로 이동 실패"
        
        #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        # 수거존 근처로 이동 (AprilTag 인식 전에, 수동 조인트값 위치)
        print("\n=== 핑키로 place를 위한 이동 ===")
        
        # 핑키 바라보는 각도
        self.mc.send_angles([-14.23, 44.56, -23.55, -49.65, -0.61, 35.59], 25)
        time.sleep(5)

        # 수거존 위치 인식
        print("\n=== 핑키 AprilTag 인식 ===")
        frame = self.camera.get_frame()
        camera_coords2, rvec_deg2, tag_id2 = detect_target(frame, target_id=pinky_id) # 타겟 id 설정

        if camera_coords2 is not None and rvec_deg2 is not None:
            try:
                base_coords2 = transform_target_pose_camera_to_base(
                    camera_coords2, rvec_deg2, self.mc.get_radians()
                )

                # 그리퍼 방향은 그대로 유지 (roll, pitch, yaw 고정)
                base_coords2[3], base_coords2[4], base_coords2[5] = -92.63, 38.84, -85.73

                # x,y,z 보정
                base_coords2[0] -= 110
                base_coords2[1] += 0
                base_coords2[2] += 7
                print(f"수거존으로 이동할 좌표: {base_coords2}")

                #경유지
                print("경유지 이동")
                self.mc.send_angles([65.47, 119.44, -111.44, -12.04, -59.5, 42.62], 25)
                time.sleep(2)

                #도착지
                print("수거존 위치로 이동 완료")
                self.mc.send_coords(base_coords2, 20, 1)
                time.sleep(2)

                # 물건 내려놓기
                print("그리퍼를 열었습니다. 물건을 내려놓았습니다.")
                self.mc.set_gripper_value(100, 50)  # 그리퍼 열기
                time.sleep(1)

                #후진
                self.mc.send_angles([65.47, 119.44, -111.44, -12.04, -59.5, 42.62], 25)
                time.sleep(2)

                # 초기 위치로 복귀
                print("초기 위치 복귀")
                self.mc.send_angles([0, 0, 0, 0, 0, 40], 25)
                time.sleep(2)
                print("작업 완료")
                
            except Exception as e:
                print(f"수거존 이동 중 오류 발생: {e}")
        else:
            print("수거존 AprilTag 인식 실패")

            # 버퍼 위치별 동작 정의 (AprilTag 인식 실패 시 fallback용)
            if pinky_id == 1:
                # 경유지 이동 (전)
                self.mc.send_angles([132.27, -32.6, -0.7, -34.89, -6.94, 47.72], 25) 
                time.sleep(3)
                
                # 버퍼로 이동
                self.mc.send_angles([129.55, -51.76, -12.12, -31.11, 0.35, 87.09], 20)
                time.sleep(3)

                # 놓기
                self.mc.set_gripper_value(100, 50)
                time.sleep(0.5)

                # 경유지 이동 (후)
                self.mc.send_angles([131.48, -14.23, -49.74, -11.33, 0.35, 93.16], 25) 
                time.sleep(2)
            elif pinky_id == 2:
                # 경유지 이동 (전)
                self.mc.send_angles([115.31, 5.27, -63.54, -7.64, 0.26, 61.61], 25)   
                time.sleep(2)
                
                # 버퍼로 이동
                self.mc.send_angles([113.55, -31.2, -54.49, -7.2, -1.84, 70.31], 20)
                time.sleep(2)

                # 놓기
                self.mc.set_gripper_value(100, 50)
                time.sleep(0.5)

                # 경유지 이동 (후)
                self.mc.send_angles([115.31, 5.27, -63.54, -7.64, 0.26, 61.61], 25) 
                time.sleep(2)
            elif pinky_id == 3:
                # 경유지 이동 (전)
                self.mc.send_angles([93.95, -29.17, -24.78, -37.44, -1.31, 51.59], 20)
                time.sleep(2)

                # 버퍼로 이동
                self.mc.send_angles([94.13, -30.67, -53.43, -7.91, -0.17, 48.69], 20)
                time.sleep(2)

                # 놓기
                self.mc.set_gripper_value(100, 50)
                time.sleep(0.5)

                # # 경유지 이동 (후)
                self.mc.send_angles([93.95, -29.17, -24.78, -37.44, -1.31, 51.59], 20)
                time.sleep(2)
            else:
                print("잘못된 핑키 번호입니다.")

            self.mc.send_angles([0, 0, 0, 0, 0, 40], 25)
            return False, '에이프릴 테그 인식 실패' # "버퍼에서 핑키로 이동 실패

        self.get_logger().info(f"버퍼 → 핑키{pinky_id}")
        return True, 'buffer_to_pinky' # "버퍼에서 핑키로 이동 완료"
    
##=========================================================================================
    '''
    RobotArm2가 pinky에서 buffer로 물건을 옮기는 함수
    '''
    def handle_pinky_to_buffer(self, pinky_id):
        # TODO: 실제 로봇 로직 작성
        
        # 핑키 바라보는 위치로 이동
        print("\n[1]: 핑키 방향으로 이동 중...")
        self.mc.send_angles([-14.23, 44.56, -23.55, -49.65, -0.61, 35.59], 25)
        self.mc.set_gripper_value(100, 50)  # 그리퍼 열기
        time.sleep(5)

        print("그리퍼를 완전히 엽니다.")
        self.mc.set_gripper_value(100, 50)
        
        # 프레임 가져오고, 프레임에서 에이프릴테그 감지
        print("\n[2] :brain: AprilTag 인식 중...")
        frame = self.camera.get_frame()
        camera_coords, rvec_deg, tag_id = detect_target(frame, target_id=id) # 타겟 id 설정 3 >> id

        if camera_coords is not None and rvec_deg is not None:
            print("\n=== April Tag 좌표 정보 ===")
            print(f"카메라 기준 좌표: {camera_coords}")
            print(f"회전 벡터 (도): {rvec_deg}")

            print("\n=== Base 좌표계로 변환 중... ===")
            try:
                base_coords = transform_target_pose_camera_to_base(
                    camera_coords, rvec_deg, self.mc.get_radians()
                )

                # roll, pitch, yaw 고정
                base_coords[3], base_coords[4], base_coords[5] = -92.77, 39.31, -87.4
                print(f"베이스 좌표 [x, y, z, roll, pitch, yaw]: {base_coords}")

                # 핑크로 가는 경유지로 이동 (1차)
                print("\n[3]: 경유지(1차) 이동 중...")
                self.mc.send_angles([99.75, 121.55, -114.08, -1.05, -101.95, 42.18], 25)
                time.sleep(3)

                # offset값 설정
                base_coords[0] -= 57
                base_coords[1] += 0
                base_coords[2] -= 10
                
                # base 기준 좌표로 이동 후 물건 잡기
                self.mc.send_coords(base_coords, 20, 1)
                time.sleep(3)
                self.mc.set_gripper_value(0, 50)  # 그리퍼 닫기
                time.sleep(1)

                # 핑키에서 후진하는 경유지
                print("\n[4]: 경유지(후진) 이동")
                self.mc.send_angles([64.59, 118.74, -111.88, -11.07, -60.38, 42.53], 30)
                time.sleep(3)

                # collection으로 이동
                print(f"\n[5]: 버퍼로 이동 중...")
                self.mc.send_angles([76.2, -45.7, -39.72, 5.8, 7.2, 26.89], 30) 
                time.sleep(3)
                
                # 놓기
                print("\n[6]: 그리퍼 열기")
                self.mc.set_gripper_value(100, 50)
                time.sleep(1)

                # 초기 위치(핑키 바라보는 방향) 복귀
                print("\n[7]: 초기 위치 복귀")
                self.mc.send_angles([-14.67, 91.58, -87.62, -37.79, -6.67, 44.2], 25)
                time.sleep(2)
                
                self.mc.send_angles([0, 0, 0, 0, 0, 40], 25)
                print("\n[8]: 작업 완료")

            except Exception as e:
                print(f"좌표 변환 또는 로봇 이동 중 오류 발생: {e}")

        else:
            print("April Tag 좌표를 가져올 수 없습니다.")
            return False, '에이프릴 테그 인식 실패' # "버퍼에서 핑키로 이동 실패"
        
        self.get_logger().info(f"핑키{pinky_id} → 버퍼")
        return True, 'pinky_to_buffer' # "핑키에서 버퍼로 이동 완료"
##=========================================================================================


def destroy_node(self):
    """노드 종료 시 리소스 정리"""
    if self.camera:
        self.camera.release()  # 카메라 해제
    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Robot2ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.camera.release()  # 안전하게 종료 시 카메라 해제
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()