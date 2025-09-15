import cv2
import numpy as np
import requests
import threading
class VideoStreamClient:
    def __init__(self, server_url, calib_file):
        self.server_url = server_url
        self.stream_active = False
        self.stream_thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        # 캘리브레이션 결과 불러오기
        calib = np.load(calib_file)
        self.K = calib['K']
        self.dist = calib['dist']
        # ArUco 설정 (5x5_250, 마커 크기 65mm)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 65  # mm
    def start_stream(self):
        if self.stream_active:
            print("이미 스트림이 활성화되어 있습니다.")
            return
        self.stream_active = True
        self.stream_thread = threading.Thread(target=self._receive_stream)
        self.stream_thread.daemon = True
        self.stream_thread.start()
    def stop_stream(self):
        self.stream_active = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()
    def _receive_stream(self):
        try:
            response = requests.get(self.server_url, stream=True)
            if response.status_code != 200:
                print(f"서버 연결 실패: {response.status_code}")
                self.stream_active = False
                return
            bytes_data = bytes()
            while self.stream_active:
                chunk = response.raw.read(1024)
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    # ArUco 마커 인식
                    corners, ids, _ = cv2.aruco.detectMarkers(
                        frame, self.aruco_dict, parameters=self.aruco_params
                    )
                    if ids is not None:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners, self.marker_length, self.K, self.dist
                        )
                        for i in range(len(ids)):
                            cv2.aruco.drawDetectedMarkers(frame, corners)
                            # drawAxis 대신 텍스트로 ID, 좌표 표시
                            c = tuple(corners[i][0][0].astype(int))
                            text = f"ID:{ids[i][0]} x:{tvecs[i][0][0]:.1f} y:{tvecs[i][0][1]:.1f} z:{tvecs[i][0][2]:.1f}"
                            cv2.putText(frame, text, (c[0], c[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                            print(f"[INFO] ID {ids[i][0]} 위치(mm): {tvecs[i][0]}")
                    with self.frame_lock:
                        self.latest_frame = frame
        except Exception as e:
            print(f"스트림 수신 중 오류 발생: {e}")
        finally:
            self.stream_active = False
if __name__ == "__main__":
    server_url = "http://192.168.5.1:5000/stream"
    calib_path = "/home/ubuntu/dev_ws/jetcobot/src/config/camera_calib_result.npz"
    client = VideoStreamClient(server_url, calib_path)
    client.start_stream()
    try:
        while True:
            frame = client.get_latest_frame()
            if frame is not None:
                cv2.imshow("ArUco Pose (No Axis)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        client.stop_stream()
        cv2.destroyAllWindows()
