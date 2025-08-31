import cv2
import numpy as np
import os
import glob
import pickle

def calibrate_camera():
    CHECKERBOARD = (8,6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []

    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    images = glob.glob('/home/jetcobot/project/calib/calib/photo_*.jpg')
    print(f"[INFO] {len(images)}장의 체커보드 이미지가 로드됨.")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH +
            cv2.CALIB_CB_FAST_CHECK +
            cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # GUI 환경에서만 코너 시각화
            if os.environ.get("DISPLAY"):
                img_disp = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
                cv2.imshow('img', img_disp)
                cv2.waitKey(100)
    if os.environ.get("DISPLAY"):
        cv2.destroyAllWindows()

    if len(objpoints) == 0 or len(imgpoints) == 0:
        raise RuntimeError("체커보드 인식에 실패했습니다. 이미지를 다시 확인하세요.")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("Camera matrix : \n", mtx)
    print("\ndist : \n", dist)

    calibration_data = {
        'camera_matrix': mtx,
        'dist_coeffs': dist,
        'rvecs': rvecs,
        'tvecs': tvecs
    }

    # 1. pkl 파일로 저장
    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump(calibration_data, f)

    # 2. npz 파일로 저장
    np.savez('camera_calibration.npz',
             camera_matrix=mtx,
             dist_coeffs=dist,
             rvecs=rvecs,
             tvecs=tvecs)

    print("[INFO] 캘리브레이션 결과가 camera_calibration.pkl, camera_calibration.npz에 모두 저장되었습니다.")

    return calibration_data

def live_video_correction(calibration_data):
    # DISPLAY 환경(GUI)에서만 실행
    if not os.environ.get("DISPLAY"):
        print("[INFO] 현재 환경에서는 영상 보정을 시각화하지 않습니다.")
        return

    mtx = calibration_data['camera_matrix']
    dist = calibration_data['dist_coeffs']
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x, y, w1, h1 = roi
        if all(v > 0 for v in [x, y, w1, h1]):
            dst = dst[y:y+h1, x:x+w1]
        original = cv2.resize(frame, (640, 480))
        corrected = cv2.resize(dst, (640, 480))
        combined = np.hstack((original, corrected))
        cv2.imshow('Original | Corrected', combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if os.path.exists('camera_calibration.pkl'):
        print("Loading existing calibration data...")
        with open('camera_calibration.pkl', 'rb') as f:
            calibration_data = pickle.load(f)
    else:
        print("Performing new camera calibration...")
        calibration_data = calibrate_camera()
    print("Starting live video correction (DISPLAY 환경에서만 실행)...")
    live_video_correction(calibration_data)
