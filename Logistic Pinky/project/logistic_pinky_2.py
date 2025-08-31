import RPi.GPIO as GPIO
import time
import cv2
from IPython.display import display, clear_output, Image
from picamera2 import Picamera2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Float32
import math
from .pinkylib import Battery  # 배터리 라이브러리

# ===============================
# 초음파 센서 클래스
# ===============================
TRIG = 23
ECHO = 24

class Ultrasonic:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        try:
            GPIO.setup(TRIG, GPIO.OUT)
            GPIO.setup(ECHO, GPIO.IN)
        except Exception as e:
            raise RuntimeError("현재 초음파 GPIO가 사용중 입니다. 사용 중인 커널을 종료 후 다시 실행해 주세요")

    def get_dist(self):
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 34300 / 2
        return distance  # cm

    def clean(self):
        GPIO.cleanup([TRIG, ECHO])

# ===============================
# PID 제어기 클래스
# ===============================
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

# ===============================
# 모터 제어 클래스
# ===============================
class MotorController:
    def __init__(self, ain1, ain2, pwma, bin1, bin2, pwmb, stby, freq=1000):
        GPIO.setmode(GPIO.BCM)
        self.pins = {
            'AIN1': ain1, 'AIN2': ain2, 'PWMA': pwma,
            'BIN1': bin1, 'BIN2': bin2, 'PWMB': pwmb,
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
        GPIO.output(self.pins['AIN1'], left_speed > 0)
        GPIO.output(self.pins['AIN2'], left_speed < 0)
        GPIO.output(self.pins['BIN1'], right_speed > 0)
        GPIO.output(self.pins['BIN2'], right_speed < 0)
        self.pwmA.ChangeDutyCycle(min(abs(left_speed), 100))
        self.pwmB.ChangeDutyCycle(min(abs(right_speed), 100))

    def stop(self):
        print("차량 정지")
        self.set_motor(0, 0)

# ===============================
# 카메라 클래스
# ===============================
class Camera:
    def __init__(self):
        self.picam2 = Picamera2()
        self.start_camera = False

    def start(self, width=640, height=480):
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (width, height)}))
        self.picam2.start()
        self.start_camera = True

    def get_frame(self):
        if not self.start_camera:
            raise RuntimeError("카메라를 시작해 주세요")
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
        binary = mask
        M = cv2.moments(binary)
        cx = width // 2
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - (width // 2)
            return binary, error
        else:
            return binary, None

# ===============================
# 라인 추종 ROS 2 노드
# ===============================
class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.cam = Camera()
        self.cam.start(640, 480)
        self.motor = MotorController(17, 27, 18, 5, 6, 13, 25)
        self.pid = PIDController(Kp=0.8, Ki=0.00001, Kd=0.05, output_limits=(-50, 50))
        self.ultrasonic = Ultrasonic()  # 초음파 센서 추가

        self.waypoints = [(0.01, 0.13), (0.3, 0.86)]
        self.current_index = 0
        self.threshold = 0.05

        self.robot_x = None
        self.robot_y = None
        self.waiting_for_start = True
        self.ready_to_go = False

        self.publisher = self.create_publisher(Int32, 'reached_point', 10)
        self.create_subscription(PoseStamped, 'cam_point_4', self.pose_callback, 10)
        self.create_subscription(Bool, 'start_signal', self.start_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def start_callback(self, msg):
        self.get_logger().info(f"📥 start_signal 수신: {msg.data}")
        if msg.data:
            self.ready_to_go = True
            self.waiting_for_start = False
            self.get_logger().info("🚦 출발 신호 수신 - 주행 재개")

    def pose_callback(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.get_logger().info(f"📍 현재 위치: x={self.robot_x:.2f}, y={self.robot_y:.2f}")

        if self.waiting_for_start:
            return

        target_x, target_y = self.waypoints[self.current_index]
        dist = math.sqrt((self.robot_x - target_x) ** 2 + (self.robot_y - target_y) ** 2)
        if dist < self.threshold:
            self.get_logger().info("✅ 웨이포인트 도달 - 정지")
            self.motor.stop()

            msg = Int32()
            msg.data = self.current_index + 1
            self.publisher.publish(msg)
            self.get_logger().info(f"📤 도달 메시지 전송: {msg.data}")

            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.ready_to_go = False
            self.waiting_for_start = True

    def control_loop(self):
        try:
            if not self.ready_to_go:
                self.get_logger().info("⏸️ 출발 대기 중... (ready_to_go=False)")
                return

            # 🔍 초음파 거리 측정
            distance = self.ultrasonic.get_dist()
            self.get_logger().info(f"📏 초음파 거리: {distance:.1f} cm")

            if distance <= 3.0:  # 3cm 이하이면 정지
                self.get_logger().warn("🛑 전방 장애물 감지 - 정지")
                self.motor.stop()
                return

            frame = self.cam.get_frame()
            _, error = self.cam.detect_line(frame)

            if error is None:
                self.get_logger().warn("⚠️ 라인 인식 실패 - 회전")
                self.motor.set_motor(-25, 25)
                time.sleep(0.4)
                return

            correction = self.pid.update(error)
            base_speed = 35
            left_speed = int(base_speed - correction / 16)
            right_speed = int(base_speed + correction / 16)
            left_speed = max(min(left_speed, 100), -100)
            right_speed = max(min(right_speed, 100), -100)

            self.motor.set_motor(left_speed, right_speed)

        except Exception as e:
            self.get_logger().error(f"🔥 control_loop 예외 발생: {e}")

# ===============================
# 배터리 퍼블리셔 ROS 2 노드
# ===============================
class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.battery = Battery()

        self.battery_publisher = self.create_publisher(
            Float32,
            '/logistic_pinky_battery_present',
            10
        )
        self.timer = self.create_timer(5.0, self.battery_callback)

    def battery_callback(self):
        msg = Float32()
        msg.data = self.battery.get_battery()
        self.battery_publisher.publish(msg)

# ===============================
# main 실행부
# ===============================
def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollowerNode()
    battery_pub = BatteryPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(line_follower)
    executor.add_node(battery_pub)

    try:
        executor.spin()
    except KeyboardInterrupt:
        line_follower.get_logger().info("🛑 종료")
    finally:
        line_follower.motor.stop()
        GPIO.cleanup()
        line_follower.ultrasonic.clean()
        line_follower.destroy_node()
        battery_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()