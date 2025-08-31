import socket
import json
import threading
HOST = '0.0.0.0'     # 모든 IP로부터 연결 허용
PORT = 8888          # GUI가 접속할 포트 (GUI와 일치시켜야 함)
def handle_client(conn, addr):
    print(f"[✓] 연결됨: {addr}")
    try:
        data = conn.recv(1024).decode('utf-8')
        if not data:
            print("[!] 수신 데이터 없음")
            return
        print(f"[⇩] 수신 데이터: {data}")
        try:
            command = json.loads(data)
            if "move" in command:
                move_state = command["move"]
                if move_state:
                    print("→ 로봇 이동 시작")
                    # 여기에 이동 로직 넣기 (ex. 모터 제어)
                else:
                    print("■ 로봇 정지")
                    # 여기에 정지 로직 넣기
            else:
                print("[!] move 키 없음")
        except json.JSONDecodeError:
            print("[!] JSON 파싱 실패")
    except Exception as e:
        print(f"[✗] 오류 발생: {e}")
    finally:
        conn.close()
        print(f"[×] 연결 종료: {addr}")
def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()
        print(f"[:톱니바퀴:] 로봇 서버 대기 중... {HOST}:{PORT}")
        while True:
            conn, addr = server_sock.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()
if __name__ == '__main__':
    start_server()