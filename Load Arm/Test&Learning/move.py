import cv2
import time
import torch
import numpy as np
from pymycobot.mycobot280 import MyCobot280


PORT = '/dev/ttyJETCOBOT'
BAUD = 1000000
CAMERA_DEV = '/dev/jetcocam0'

mc = MyCobot280(PORT, BAUD)
cap = cv2.VideoCapture(CAMERA_DEV)
cap.set(cv2.CAP_PROP_FPS, 15)

calib = np.load('/home/jetcobot/project/calib/calibration.npz')
K = calib['K']
f = K[0, 0]
W = 30

model = torch.hub.load(
    '/home/jetcobot/project/calib/src/yolov5',
    'custom',
    path='/home/jetcobot/project/calib/src/yolov5/runs/train/exp15/weights/best.pt',
    source='local'
)

points_world = np.array([[15.8, 51.4], [-32.9, 22.9], [-45.7, -20.9], [27.8, -20.4]], dtype=np.float32)
points_img = np.array([[370, 395], [22, 388], [200, 93], [488, 104]], dtype=np.float32)
H, _ = cv2.findHomography(points_img, points_world)

print("[INFO] 초기 자세 이동")
mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
mc.set_gripper_value(70, 50)
coords = mc.get_coords()

print("[INFO] 메인 루프 시작 (q 누르면 종료)")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 카메라 프레임 읽기 실패")
            time.sleep(0.1)
            continue

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model(img_rgb)

        detected = False
        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            x1, y1, x2, y2 = map(int, box)
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            box_w = x2 - x1
            cz = (f * W) / box_w
            print(f"Pixel 좌표계: X:{cx:.1f}, Y:{cy:.1f}")

            pixel = np.array([[[cx, cy]]], dtype='float32')
            real = cv2.perspectiveTransform(pixel, H)
            X_mm, Y_mm = real[0][0]
            X_mm -= 20
            Y_mm += 20
            print(f"World 좌표계: X:{X_mm:.1f}, Y:{Y_mm:.1f}, Z:{cz:.1f}")

            # ====== 바운딩박스 그리기 ======
            label_name = model.names[int(cls)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{label_name} {conf:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # ====== 감지된 이미지 저장 ======
            save_path = f'./detected_{label_name}_{time.strftime("%Y%m%d_%H%M%S")}.jpg'
            cv2.imwrite(save_path, frame)
            print(f"[INFO] 감지 이미지 저장 완료: {save_path}")

            goal_coords = coords.copy()
            if (Y_mm > 50 and X_mm > -30) or (Y_mm > 40 and X_mm < -29):
                goal_coords[2] = 345
                goal_coords[0] = -15
                if (X_mm < -35):
                    goal_coords[1] -= X_mm + 10
                    print("1")
                elif (X_mm < -26) :
                    goal_coords[1] -= X_mm + 15
                    print("2")
                elif (X_mm < 6) :
                    goal_coords[1] -= X_mm + 20
                    print("3")
                elif (X_mm < 30):
                    goal_coords[1] -= X_mm + 12
                    print("4")
                else :
                    goal_coords[1] -= X_mm + 3
                    print("5")

            else:
                goal_coords[2] = 385
                goal_coords[0] = -15
                if (X_mm < - 53):
                    goal_coords[1] -= X_mm + 18
                    print("1")
                elif (X_mm > -58 and X_mm < -42) :
                    goal_coords[1] -= X_mm + 30
                    print("2")
                elif (X_mm < 5) :
                    goal_coords[1] -= X_mm + 35
                    print("3")
                elif (X_mm < 14) :
                    goal_coords[1] -= X_mm + 21
                    print("4")
                else :
                    goal_coords[1] -= X_mm + 17
                    print("5")

            mc.send_coords(goal_coords, 50, 1)
            time.sleep(2)
            
            goal_coords[0] += 5
            mc.send_coords(goal_coords, 50, 1)
            goal_coords[0] += 10
            mc.send_coords(goal_coords, 50, 1)
            time.sleep(1)

            mc.set_gripper_value(0, 50)
            time.sleep(1)

            mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
            mc.set_gripper_value(70, 50)
            time.sleep(1)

            detected = True
            break

        if detected:
            break

except KeyboardInterrupt:
    print("[INFO] 종료 요청됨")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] 카메라 해제 및 프로그램 종료")
