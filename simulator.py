from RobotBase import RobotBase
import time
import math
import asyncio
from TelemetryManager import TelemetryManager

# Adapted from https://stackoverflow.com/a/20677983
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return False, 0, 0

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    #TODO fix "on line" detection
    #if ((x > max(line1[0][0], line1[1][0]) or x < min(line1[0][0], line1[1][0])) or
    #   (y > max(line1[0][1], line1[1][1]) or y < min(line1[0][1], line1[1][1])) or
    #   (x > max(line2[0][0], line2[1][0]) or x < min(line2[0][0], line2[1][0])) or
    #   (y > max(line2[0][1], line2[1][1]) or y < min(line2[0][1], line2[1][1]))):
    #    return False, x, y

    return True, x, y


def distance_between_points(p1, p2):
    return math.sqrt(((p1[0]-p2[0])**2) + ((p1[1]-p2[1])**2))


class Simulator:
    telemetry = None
    robot = RobotBase()

    ultrasonic_values = [100, 100, 100, 100, 100]
    pos = (0.000, 0.000)
    #pos = (0.000, 0.500)
    #angle = (math.pi * 1.9)
    angle = 0
    lines = [
        ((1500, 1500), (1500, -1500)),
        ((1500, -1500), (-1500, -1500)),
        ((-1500, -1500), (-1500, 1500)),
        ((-1500, 1500), (1500, 1500)),
             ]

    def __init__(self):
        None

    async def start(self):
        self.telemetry = TelemetryManager(9090)

        await asyncio.gather(
            self.telemetry.start(),
            self._loop()
        )

    async def _loop(self):
        await asyncio.sleep(1.0)
        while True:
            dots = []
            ultrasonic_range = 1700
            ultrasonic_offset = 140
            for i in range(0, 5):
                ultrasonic_angle = (((self.angle * -1) - (math.pi / 2)) + (math.pi - ((math.pi / 4) * i)) + (math.pi / 2)) % (math.pi * 2)
                ultrasonic_line = (((self.pos[0] * 1000) + (ultrasonic_offset * math.cos(ultrasonic_angle)),
                                    (self.pos[1] * 1000) + (ultrasonic_offset * math.sin(ultrasonic_angle))),
                                   ((self.pos[0] * 1000) + (ultrasonic_range * math.cos(ultrasonic_angle)),
                                    (self.pos[1] * 1000) + (ultrasonic_range * math.sin(ultrasonic_angle)))
                                    )
                dots.append(ultrasonic_line[0])
                dots.append(ultrasonic_line[1])
                has_collision = False
                smallest_collision = 9999
                for line in self.lines:
                    collides, x, y = line_intersection(ultrasonic_line, line)
                    if collides:
                        dots.append((x, y))
                        dots.append(ultrasonic_line[0])
                        has_collision = True
                        smallest_collision = min(smallest_collision, distance_between_points(ultrasonic_line[0], (x, y)))
                if has_collision:
                    self.ultrasonic_values[i] = (smallest_collision / 10)
                else:
                    self.ultrasonic_values[i] = (ultrasonic_range / 10)

            self.telemetry.frame['dots'] = dots

            self.robot.set_position(self.pos, self.angle)
            self.robot.set_ultrasonic(self.ultrasonic_values)

            v, w = self.robot.get_velocity(self.telemetry.frame)

            v = v / 30
            w = w / 100

            self.angle = (self.angle + w) % (math.pi * 2)
            self.pos = (self.pos[0] + (v * math.cos(self.angle - (math.pi / 2))), self.pos[1] + (v * math.sin(self.angle - (math.pi / 2))))

            self.telemetry.frame['lines'] = self.lines
            self.telemetry.send_frame()

            await asyncio.sleep(0.1)


if __name__ == '__main__':
    simulator = Simulator()
    try:
        asyncio.get_event_loop().run_until_complete(simulator.start())
    except KeyboardInterrupt:
        print('Killed by interrupt')
