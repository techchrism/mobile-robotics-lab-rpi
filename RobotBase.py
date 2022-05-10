

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
        v = 0.0
        w = 0.0

        v = 0.5

        return v, w
