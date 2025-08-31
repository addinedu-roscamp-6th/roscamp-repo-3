import cv2
from IPython.display import display, clear_output, Image
import time
from picamera2 import Picamera2
import numpy as np
import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node

# PID 제어기 클래스
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0
        self._integral = 0
        self.output_limits = output_limits
        self._last_time = None

    def reset(self):
        self._last_error = 0
        self._integral = 0
        self._last_time = None

    def update(self, current_value):
        error = self.setpoint - current_value
        current_time = time.time()
        delta_time = current_time - self._last_time if self._last_time is not None else 0
        delta_error = error - self._last_error

        if delta_time > 0:
            self._integral += error * delta_time
            derivative = delta_error / delta_time
        else:
            derivative = 0

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        low, high = self.output_limits
        if low is not None and output < low:
            output = low
        if high is not None and output > high:
            output = high

        self._last_error = error
        self._last_time = current_time

        return output

# 모터 제어 클래스
class MotorController:
    def __init__(self, ain1, ain2, pwma, bin1, bin2, pwmb, stby, freq=1000):
        GPIO.setmode(GPIO.BCM)
        self.pins = {
            'AIN1': ain1,
            'AIN2': ain2,
            'PWMA': pwma,
            'BIN1': bin1,
            'BIN2': bin2,
            'PWMB': pwmb,
            'STBY': stby
        }
        for pin in self.pins.values():
            GPIO.setup(pin, GPIO.OUT)
        self.pwmA = GPIO.PWM(self.pins['PWMA'], freq)
        self.pwmB = GPIO.PWM(self.pins['PWMB'], freq)
        self.pwmA.start(0)
        self.pwmB.start(0)
        self.den = 8
        GPIO.output(self.pins['STBY'], GPIO.HIGH)

    def set_motor(self, left_speed, right_speed):
        if left_speed > 0:
            GPIO.output(self.pins['AIN1'], GPIO.HIGH)
            GPIO.output(self.pins['AIN2'], GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(self.pins['AIN1'], GPIO.LOW)
            GPIO.output(self.pins['AIN2'], GPIO.HIGH)
        else:
            GPIO.output(self.pins['AIN1'], GPIO.LOW)
            GPIO.output(self.pins['AIN2'], GPIO.LOW)
        self.pwmA.ChangeDutyCycle(min(abs(left_speed), 100))

        if right_speed > 0:
            GPIO.output(self.pins['BIN1'], GPIO.HIGH)
            GPIO.output(self.pins['BIN2'], GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(self.pins['BIN1'], GPIO.LOW)
            GPIO.output(self.pins['BIN2'], GPIO.HIGH)
        else:
            GPIO.output(self.pins['BIN1'], GPIO.LOW)
            GPIO.output(self.pins['BIN2'], GPIO.LOW)
        self.pwmB.ChangeDutyCycle(min(abs(right_speed), 100))

    def stop(self):
        self.set_motor(0, 0)
        GPIO.output(self.pins['STBY'], GPIO.LOW)
        self.pwmA.stop()
        self.pwmB.stop()
        GPIO.cleanup()

# 카메라 클래스
class Camera:
    def __init__(self):
        try:
            self.picam2 = Picamera2()
        except:
            raise RuntimeError("현재 카메라가 사용 중입니다. 사용 중인 커널을 종료 후 다시 실행해 주세요")

        self.start_camera = False

    def start(self, width=640, height=480):
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (width, height)}))
        self.picam2.start()
        self.start_camera = True

    def get_frame(self):
        if not self.start_camera:
            raise RuntimeError("카메라를 시작해 주세요")
        frame = self.picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def display_jupyter(self, frame):
        _, buffer = cv2.imencode('.jpg', frame)
        clear_output(wait=True)
        display(Image(data=buffer, width=500))

    def detect_line(self, frame):
        height, width, _ = frame.shape
        roi = frame[int(height*0.7):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        binary = mask
        M = cv2.moments(binary)
        cx = width // 2
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - (width // 2)
            return binary, error
        else:
            return binary, None  # 라인 인식 실패

# 라인트레이싱
def follow_line(pid, motor, cam, base_speed=35, duration=30):
    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            frame = cam.get_frame()
            _, error = cam.detect_line(frame)

            if error is None:
                print("[!] 라인 인식 실패 - 왼쪽으로 3초 회전")
                motor.set_motor(-30, 30)
                time.sleep(0.58)
                motor.set_motor(0, 0)
                time.sleep(2)
                continue

            correction = pid.update(error)

            left_speed = int((base_speed - correction / 16))
            right_speed = int((base_speed + correction / 16))

            left_speed = max(min(left_speed, 100), -100)
            right_speed = max(min(right_speed, 100), -100)

            motor.set_motor(left_speed, right_speed)
            cam.display_jupyter(frame)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        print("라인 추적 종료, 모터 정지")

# ROS2 노드 클래스
class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        cam = Camera()
        cam.start(640, 480)
        motor = MotorController(17, 27, 18, 5, 6, 13, 25)
        pid = PIDController(Kp=0.8, Ki=0.00001, Kd=0.05, output_limits=(-50, 50))
        follow_line(pid, motor, cam, base_speed=35, duration=30)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.shutdown()
