"""
로봇팔에 부착된 카메라를 불러서 이미지를 찍는 모듈
"""
# import cv2
# import time

# class CameraManager:
#     def __init__(self, device='/dev/jetcocam0'):
#         self.cap = cv2.VideoCapture(device)
#         if not self.cap.isOpened():
#             raise RuntimeError(f"카메라를 열 수 없습니다: {device}")

#     # def get_frame(self):
#     #     ret, frame = self.cap.read()
#     #     if not ret:
#     #         raise RuntimeError("프레임을 읽지 못했습니다.")
#     #     # return frame
#     #     return ret, frame

#     def get_frame(self):
#         # 버퍼 완전히 비우기 - 최대 10프레임까지만 시도
#         for _ in range(10):
#             ret = self.cap.grab()
#             if not ret:
#                 break
        
#         # 새로운 프레임을 위한 대기
#         time.sleep(0.1)  # 100ms 대기
        
#         # 최신 프레임 읽기
#         ret, frame = self.cap.read()
#         if not ret:
#             raise RuntimeError("프레임을 읽지 못했습니다.")
        
#         return frame
        
#     def destroy_node(self):
#         self.camera.release()  # 카메라 해제
#         super().destroy_node()

#     def release(self): # Note: destroy_node와 차이점은???????????????
#         if self.cap:
#             self.cap.release()


import cv2
import threading
import time

class CameraManager:
    def __init__(self, device='/dev/jetcocam0'):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: {device}")

        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

    def _update_frame(self):
        """카메라로부터 프레임을 계속 읽는 백그라운드 루프"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            time.sleep(0.03)  # 30 FPS 가정 (너무 높으면 부하 발생)

    def get_frame(self):
        """가장 최근 프레임을 반환"""
        with self.lock:
            if self.latest_frame is None:
                raise RuntimeError("아직 프레임이 준비되지 않았습니다.")
            return self.latest_frame.copy()

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()