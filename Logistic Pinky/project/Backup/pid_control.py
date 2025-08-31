import time


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