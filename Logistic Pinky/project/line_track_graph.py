#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import time
from picamera2 import Picamera2
import numpy as np
import RPi.GPIO as GPIO
from std_msgs.msg import String  # 예시용, 필요 시 제거 가능
import matplotlib.pyplot as plt


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

        # 출력 제한
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
            raise RuntimeError("현재 카메라가 사용 중입니다.")
        self.start_camera = False

    def start(self, width=640, height=480):
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (width, height)}))
        self.picam2.start()
        self.start_camera = True

    def get_frame(self):
        if not self.start_camera:
            raise RuntimeError("카메라 시작되지 않음")
        frame = self.picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def detect_line(self, frame):
        height, width, _ = frame.shape
        roi = frame[int(height*0.7):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        M = cv2.moments(mask)
        cx = width // 2
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
        error = cx - (width // 2)
        print(f"error = {error}")
        return mask, error


# ROS 2 노드
class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info("라인 트레이서 노드 시작")

        # 핀 설정
        AIN1 = 17
        AIN2 = 27
        PWMA = 18
        BIN1 = 5
        BIN2 = 6
        PWMB = 13
        STBY = 25

        self.cam = Camera()
        self.cam.start(640, 480)

        self.motor = MotorController(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY)
        self.pid = PIDController(Kp=0.35, Ki=0.00001, Kd=0.05, output_limits=(-50, 50))

        # --- 그래프 초기화 ---
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line_error, = self.ax.plot([], [], "b-", label="Error")        # 파란색 라인
        self.line_correction, = self.ax.plot([], [], "r-", label="PID Output")  # 빨간색 라인
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlabel("Frame")
        self.ax.set_ylabel("Value")
        self.ax.legend()
        self.error_vals, self.correction_vals, self.x_vals = [], [], []
        self.frame_count = 0

        plt.show(block=False)

        # 타이머
        self.timer = self.create_timer(0.05, self.follow_line)
        self.start_time = time.time()
        self.duration = 30  # 초

    def update_graph(self, error, correction):
        self.x_vals.append(self.frame_count)
        self.error_vals.append(error)
        self.correction_vals.append(correction)

        self.line_error.set_data(self.x_vals, self.error_vals)
        self.line_correction.set_data(self.x_vals, self.correction_vals)

        self.ax.set_xlim(max(0, self.frame_count - 100), self.frame_count + 10)
        self.frame_count += 1

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def follow_line(self):
        if time.time() - self.start_time > self.duration:
            self.motor.stop()
            self.get_logger().info("라인 추적 종료")
            rclpy.shutdown()
            return

        frame = self.cam.get_frame()
        _, error = self.cam.detect_line(frame)
        correction = self.pid.update(error)

        # 그래프 갱신 (error와 correction 동시에)
        self.update_graph(error, correction)

        base_speed = 23
        left_speed = int(base_speed - correction)
        right_speed = int(base_speed + correction)

        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        self.motor.set_motor(left_speed, right_speed)



# main
def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
