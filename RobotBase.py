import math


class RobotBase:
    def __init__(self):
        self.position = (0.0, 0.0)
        self.angle = 0.0
        self.ultrasonic = (0.0, 0.0, 0.0, 0.0, 0.0)

    def set_position(self, position, angle):
        self.position = position
        self.angle = angle

    def set_ultrasonic(self, ultrasonic):
        self.ultrasonic = ultrasonic

    def get_velocity(self):
        coef = 0.477465

        v = 0.0
        w = 1.5 * math.pi

        # Hardcoded bc reasons
        goal = (0, 0)

        # Derive our displacement from where we want to go
        pos_error = (goal[0] - self.position[0], goal[1] - self.position[1])
        print(f"Pos Err: {pos_error}")

        # Derive distance to angle we *should* be at
        angle_error = math.atan2(pos_error[1], pos_error[0]) - self.angle
        print(f"Ang Err: {angle_error * 180 / math.pi}")

        # Our angular velocity should scale down as we get closer
        # For this we choose w = a * t^2 for a nice quadratic
        # a is found to place our maximum velocity at maximum error
        w = coef * (angle_error ** 2)

        naive_v = 0

        return v, w
