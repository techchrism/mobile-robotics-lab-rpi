from OptitrackManager import OptitrackManager
from ArduinoManager import ArduinoManager
from RobotBase import RobotBase
import time
import threading
import math
import asyncio
from TelemetryManager import TelemetryManager
import transformations as tf


optitrack_data = None
optitrack_id = 25
ultrasonic_data = None
robot = RobotBase()


def current_milli_time():
    return round(time.time() * 1000)


last_time = current_milli_time()


def q_to_angle(orient):
    euler = tf.euler_from_quaternion(orient)
    angle = euler[1]
    if(abs(euler[2]) < 2 or angle < 0):
        angle += ((math.pi / 2) - angle) * 2
    if(euler[1] < 0 and abs(euler[2]) > 2):
        angle += ((math.pi / 2) - abs(euler[1])) * 2
    return angle


class Coordinator:
    arduino = None
    optitrack = None
    telemetry = None

    ultrasonic_values = None
    optitrack_data = None

    def __init__(self):
        None

    def _ultrasonic_callback(self, values):
        values = [x or 999 for x in values]
        self.ultrasonic_values = values

    def _optitrack_callback(self, bodies, markers, timing):
        optitrack_id = 25
        for body in bodies:
            if body.id_ != optitrack_id:
                continue
            self.optitrack_data = body
            #print(f'{body.position[0]}\t{body.position[1]}\t{body.position[2]}\t')

    async def start(self):
        self.arduino = ArduinoManager('/dev/ttyACM0', self._ultrasonic_callback)
        self.optitrack = OptitrackManager(self._optitrack_callback)
        self.telemetry = TelemetryManager(9090)

        await asyncio.gather(
            self.arduino.start(),
            self.optitrack.start(),
            self.telemetry.start(),
            self._loop()
        )

    async def _loop(self):
        await asyncio.sleep(1.0)
        while True:
            if self.optitrack_data is None:
                await asyncio.sleep(0.1)
                continue

            angle = q_to_angle(self.optitrack_data.orientation)
            angle = (angle + math.pi) % (math.pi * 2)
            robot.set_position((self.optitrack_data.position[0] * -1, self.optitrack_data.position[2] * -1), angle)
            robot.set_ultrasonic(self.ultrasonic_values)
            #robot.set_ultrasonic([200, 200, 200, 200, 200])

            v, w = robot.get_velocity(self.telemetry.frame)

            #self.arduino.send_speed(v, w)
            self.telemetry.send_frame()

            await asyncio.sleep(0.1)

    def kill(self):
        #self.arduino.send_kill()
        None


if __name__ == '__main__':
    coordinator = Coordinator()
    try:
        asyncio.get_event_loop().run_until_complete(coordinator.start())
    except KeyboardInterrupt:
        coordinator.kill()
        print('Killed by interrupt')
