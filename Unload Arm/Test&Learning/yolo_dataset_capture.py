import cv2
import os

# 저장할 디렉토리
save_dir = './dataset/images'
os.makedirs(save_dir, exist_ok=True)

# 사용할 카메라 디바이스 경로 (적절히 변경 필요)
camera_path = '/dev/jetcocam0'  # 또는 '/dev/video0' 등

# 카메라 열기
cap = cv2.VideoCapture(camera_path)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

count = 0
print("스페이스바를 눌러 이미지를 저장하고, q를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break

    # 실시간 화면 보여주기
    cv2.imshow("JetCobot Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):  # 스페이스바 누르면 저장
        filename = os.path.join(save_dir, f'image_{count:04d}.jpg')
        cv2.imwrite(filename, frame)
        print(f"{filename} 저장됨.")
        count += 1

    elif key == ord('q'):  # q 누르면 종료
        print("종료합니다.")
        break

cap.release()
cv2.destroyAllWindows()
