import socket
import json
import datetime
import threading
import os
import csv
from collections import defaultdict

HOST, PORT = "0.0.0.0", 5010
BUFF = 4096
CSV_FILE = "received_data.csv"

# 한글 → 영어 매핑 사전
PRODUCT_TRANSLATIONS = {
    "과자": "snack",
    "음료": "drink",
    "채소": "vegetable",
    "신발": "shoes"
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

            # 같은 ID, 같은 시간대의 주문을 묶기 위한 딕셔너리
            grouped_orders = defaultdict(lambda: {"items": [], "counts": []})

            for o in orders:
                product_en = translate_product(o["product"])
                grouped_orders[o["id"]]["items"].append(product_en)
                grouped_orders[o["id"]]["counts"].append(o["quantity"])

            with open(CSV_FILE, "a", encoding="utf-8", newline="") as f:
                writer = csv.writer(f)
                for order_id, data in grouped_orders.items():
                    # 여러 품목은 리스트 → 문자열 변환하여 저장
                    items_str = json.dumps(data["items"], ensure_ascii=False)
                    counts_str = json.dumps(data["counts"], ensure_ascii=False)

                    writer.writerow([now, order_id, items_str, counts_str, 3])

            print(f"[{addr}] 저장 완료 ({len(grouped_orders)}건)")

        except Exception as e:
            print(f"[{addr}] JSON 파싱 오류:", e)

def main():
    ensure_csv_header()
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
