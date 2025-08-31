import socket
import json
import datetime
import threading
import os
import csv

HOST, PORT = "0.0.0.0", 5010
BUFF = 4096
CSV_FILE = "received_data.csv"

# 한글 → 영어 매핑 사전
PRODUCT_TRANSLATIONS = {
    "과자": "snack",
    "음료": "drink",
    "채소": "vegetable",
    "신발": "fruit"
}

# 물품명 변환 함수
def translate_product(name):
    return PRODUCT_TRANSLATIONS.get(name, name)  # 사전에 없는 경우 원래 이름 그대로

# 처음 실행 시 헤더가 없으면 헤더 추가
def ensure_csv_header():
    if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
        with open(CSV_FILE, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["datetime", "ID", "Item", "Count", "Status"])

def handle_client(conn, addr):
    print(f"[{addr}] 접속")
    buf = b""
    with conn:
        while chunk := conn.recv(BUFF):
            if not chunk:
                break
            buf += chunk

        try:
            orders = json.loads(buf.decode())
            if isinstance(orders, dict):
                orders = [orders]

            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            with open(CSV_FILE, "a", encoding="utf-8", newline="") as f:
                writer = csv.writer(f)
                for o in orders:
                    # 물품명 영어로 변환
                    product_en = translate_product(o["product"])
                    writer.writerow([now, o["id"], product_en, o["quantity"], 2])

                    # status가 1일 경우 클라이언트에게 데이터 전송
                    if o.get("status") == 1:
                        response = {
                            "id": o["id"],
                            "item": product_en,
                            "count": o["quantity"]
                        }
                        conn.sendall(json.dumps(response).encode('utf-8'))

            print(f"[{addr}] 저장 완료 ({len(orders)}건)")

        except Exception as e:
            print(f"[{addr}] JSON 파싱 오류:", e)

def main():
    ensure_csv_header()  # 서버 시작 시 헤더 체크
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"주문 서버 대기 중… ({HOST}:{PORT})")
        while True:
            conn, addr = s.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()
