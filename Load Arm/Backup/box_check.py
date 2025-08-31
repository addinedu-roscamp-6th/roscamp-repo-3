import cv2
import torch

cap = cv2.VideoCapture('/dev/jetcocam0')
ret, frame = cap.read()
print('frame:', frame.shape, frame.dtype)
cv2.imwrite('test_frame.jpg', frame)

# 반드시 detect.py에서 사용한 best.pt 경로!
model = torch.hub.load(
    '/home/jetcobot/project/calib/src/yolov5',
    'custom',
    path='/home/jetcobot/project/calib/src/yolov5/runs/train/exp8/weights/best.pt',
    source='local'
)

# BGR → RGB 변환
rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
results = model(rgb)
print('YOLO result:', results.xyxy[0])

for *box, conf, cls in results.xyxy[0].cpu().numpy():
    print('box:', box, 'conf:', conf, 'cls:', cls)
