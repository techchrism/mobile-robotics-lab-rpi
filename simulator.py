from RobotBase import RobotBase
import time
import math
import asyncio
from TelemetryManager import TelemetryManager

# From https://stackoverflow.com/a/20677983
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


class Simulator:
    telemetry = None
    robot = RobotBase()

    ultrasonic_values = [100, 100, 100, 100, 100]
    pos = (0, 0)
    angle = 0

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
            self.robot.set_position(self.pos, self.angle)
            self.robot.set_ultrasonic(self.ultrasonic_values)

            v, w = self.robot.get_velocity(self.telemetry.frame)

            self.angle += w
            self.pos = (self.pos[0] + (v * math.cos(self.angle - (math.pi / 2))), self.pos[1] + (v * math.sin(self.angle - (math.pi / 2))))

            self.telemetry.send_frame()

            await asyncio.sleep(0.1)


if __name__ == '__main__':
    #simulator = Simulator()
    #try:
    #    asyncio.get_event_loop().run_until_complete(simulator.start())
    #except KeyboardInterrupt:
    #    print('Killed by interrupt')

    print(line_intersection(((0, 0), (3, 0)), ((0, 3), (3, 3))))

