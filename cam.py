import cv2
import numpy as np
import requests
import threading
import time
import os
import re

class VideoStreamClient:
    def __init__(self, server_url):
        self.server_url = server_url
        self.stream_active = False
        self.stream_thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()

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

                    with self.frame_lock:
                        self.latest_frame = frame
        except Exception as e:
            print(f"스트림 수신 중 오류 발생: {e}")
        finally:
            self.stream_active = False

if __name__ == "__main__":
    server_url = "http://192.168.1.17:5000/stream"
    client = VideoStreamClient(server_url)
    client.start_stream()

    # 저장 디렉토리 지정 (필요시 변경)
    save_dir = "./"
    os.makedirs(save_dir, exist_ok=True)

    try:
        while True:
            frame = client.get_latest_frame()
            if frame is not None:
                cv2.imshow("JetCobot Camera Stream", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    filename = os.path.join(save_dir, "vegetabel.jpg")
                    cv2.imwrite(filename, frame)
                    print(f"[INFO] 사진 저장: {filename}")
                elif key == ord('q'):
                    print("프로그램 종료")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        client.stop_stream()
        cv2.destroyAllWindows()
