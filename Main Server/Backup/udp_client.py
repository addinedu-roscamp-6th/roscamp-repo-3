# sender.py
import cv2
import socket
import numpy as np

UDP_IP = "192.168.1.11"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(2)  # 또는 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
    data = buffer.tobytes()

    print(f"전송 중: {len(data)} bytes")
    
    # UDP는 데이터 크기 제한 있음 (보통 64KB 이내). 나누어서 전송해야 안전.
    for i in range(0, len(data), 60000):
        sock.sendto(data[i:i+60000], (UDP_IP, UDP_PORT))

        

