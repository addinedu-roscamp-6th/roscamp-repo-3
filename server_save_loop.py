# server_save_loop.py
import socket, json, datetime, threading

HOST, PORT = "0.0.0.0", 9999
BUFF = 4096

def handle_client(conn, addr):
    print(f"[{addr}] 접속")
    buf = b""
    with conn:
        while chunk := conn.recv(BUFF):
            buf += chunk
        try:
            orders = json.loads(buf.decode())
            # orders 가 리스트인지 단건인지 모두 처리
            if isinstance(orders, dict):
                orders = [orders]

            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open("received_data.txt", "a", encoding="utf-8") as f:
                for o in orders:
                    f.write(f"[{now}] ID:{o['id']} | 물품:{o['product']} | 수량:{o['quantity']}\n")
            print(f"[{addr}] 저장 완료 ({len(orders)}건)")
        except Exception as e:
            print(f"[{addr}] JSON 파싱 오류:", e)

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"◆ 주문 서버 대기 중… ({HOST}:{PORT})")

        while True:                       # ← 리스너 무한루프
            conn, addr = s.accept()
            threading.Thread(target=handle_client,
                             args=(conn, addr),
                             daemon=True).start()

if __name__ == "__main__":
    main()
