import time


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._last_time = -1
        self._previous_error = 0
        self._integral = 0

    def calculate(self, input_value, target_value):
        now = time.time()
        dt = now - self._last_time if self._last_time != -1 else 0.01

        error = target_value - input_value
        e_p = error
        e_d = (error - self._previous_error) / dt
        e_i = self._integral + (error * dt)

        value = (self.kp * e_p) + (self.ki + e_i) + (self.kd + e_d)

        self._integral = e_i
        self._previous_error = error
        self._last_time = now
