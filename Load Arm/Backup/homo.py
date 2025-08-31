import cv2
import numpy as np

image = cv2.imread('/home/jetcobot/project/calib/photo_20250723_140040.jpg')

# 이미지에서 기준점 4개 이상을 수동으로 마우스로 클릭
points_img = []

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points_img.append((x, y))
        print(f"이미지 기준점: {x}, {y}")
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow('img', image)

cv2.imshow('img', image)
cv2.setMouseCallback('img', on_mouse)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(points_img)
