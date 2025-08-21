import cv2
import threading
import time
from flask import Flask, Response

class CameraManager:
    def __init__(
        self,
        device='/dev/jetcocam0',
        enable_streaming=True,
        flask_port=5000,
        jpeg_quality=80,         # JPEG 품질 (기본 80으로 CPU 부담 완화)
        stale_timeout=0.7,       # 프레임 신선도 타임아웃(초)
        fail_threshold=20,       # read 연속 실패 횟수 임계(예: 20회 ≈ 1초)
    ):
        self.device = device
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: {self.device}")

        # 공유 상태
        self.latest_frame = None
        self.last_ts = 0.0
        self.running = True
        self.lock = threading.Lock()

        # 파라미터
        self.jpeg_quality = int(jpeg_quality)
        self.stale_timeout = float(stale_timeout)
        self.fail_threshold = int(fail_threshold)

        # 워치독 카운터
        self.fail_count = 0

        # 프레임 캡처 스레드 시작 (원본만 저장)
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

        # Flask 스트리밍 (선택)
        self.enable_streaming = bool(enable_streaming)
        if self.enable_streaming:
            self.app = Flask(__name__)
            self._setup_routes()
            self.flask_thread = threading.Thread(
                target=lambda: self.app.run(
                    host='0.0.0.0',
                    port=flask_port,
                    threaded=True,
                    debug=False,
                    use_reloader=False
                ),
                daemon=True
            )
            self.flask_thread.start()
            print(f"카메라 스트리밍 시작: http://0.0.0.0:{flask_port}/stream")

    # ──────────────────────────────
    # 캡처 스레드: 원본 프레임만 갱신
    # ──────────────────────────────
    def _update_frame(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame  # copy 불필요: get_frame/stream에서 복사
                    self.last_ts = time.time()
                self.fail_count = 0
            else:
                # 연속 실패 카운트 증가 후 일정 횟수 이상이면 재오픈 시도
                self.fail_count += 1
                if self.fail_count >= self.fail_threshold:
                    self._reopen_camera()
                    self.fail_count = 0
                # 실패 시 짧게 쉼
                time.sleep(0.05)

            # 과도한 CPU 사용 방지 (read/lock만 하는 가벼운 루프)
            time.sleep(0.001)

    # ──────────────────────────────
    # 카메라 재오픈 (워치독)
    # ──────────────────────────────
    def _reopen_camera(self):
        try:
            print("[CameraManager] read 실패 연속 발생 → 카메라 재오픈 시도")
            try:
                self.cap.release()
            except Exception:
                pass
            time.sleep(0.2)
            cap = cv2.VideoCapture(self.device)
            if not cap.isOpened():
                print("[CameraManager] 재오픈 실패: 장치가 열리지 않습니다")
                return
            self.cap = cap
            print("[CameraManager] 재오픈 성공")
        except Exception as e:
            print(f"[CameraManager] 카메라 재오픈 중 예외: {e}")

    # ──────────────────────────────
    # 최신 프레임 획득 (ROS/로봇용)
    # ──────────────────────────────
    def get_frame(self, require_fresh=True):
        with self.lock:
            if self.latest_frame is None:
                raise RuntimeError("아직 프레임이 준비되지 않았습니다.")
            age = time.time() - self.last_ts
            if require_fresh and age > self.stale_timeout:
                raise RuntimeError(f"프레임이 {age:.2f}s 동안 갱신되지 않았습니다.")
            # 호출자 쪽에서 안전하게 사용할 수 있도록 복사본 반환
            return self.latest_frame.copy()

    # ──────────────────────────────
    # Flask 라우트
    # ──────────────────────────────
    def _setup_routes(self):
        @self.app.route('/stream')
        def video_feed():
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @self.app.route('/')
        def index():
            return '''
            <html><head><title>Camera Stream</title></head>
            <body style="text-align:center; font-family:Arial;">
            <h1>Robot Camera Stream</h1>
            <img src="/stream" style="max-width:90%; border:2px solid #ddd;">
            </body></html>
            '''

    # ──────────────────────────────
    # 스트리밍용 프레임 생성 (온디맨드 JPEG 인코딩)
    # ──────────────────────────────
    def _generate_frames(self):
        while self.running:
            # 최신 프레임 스냅샷
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is None:
                time.sleep(0.1)
                continue

            # 여기서 인코딩 → 캡처 스레드와 분리되어 캡처 지연 없음
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                # 인코딩 실패 시 다음 프레임 시도
                time.sleep(0.01)
                continue

            frame_data = buf.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

            # 클라이언트가 매우 느릴 때 서버 과부하 방지용 짧은 슬립
            time.sleep(0.001)

    # ──────────────────────────────
    # 자원 해제
    # ──────────────────────────────
    def release(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        try:
            self.cap.release()
        except Exception:
            pass
        # Flask dev 서버는 daemon 스레드라 프로세스 종료 시 같이 내려감
