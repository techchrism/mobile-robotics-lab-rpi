from OptitrackManager import OptitrackManager
from ArduinoManager import ArduinoManager
from RobotBase import RobotBase
import time
import threading
import math
import asyncio
#from pyquaternion import Quaternion
#import quaternionTest
from TelemetryManager import TelemetryManager

optitrack_data = None
optitrack_id = 25
ultrasonic_data = None
robot = RobotBase()


def current_milli_time():
    return round(time.time() * 1000)


last_time = current_milli_time()


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z


def quaternion_to_angle(q):
    #trident = math.atan2(2*((q[0]*q[3]) + (q[1]*q[2])), 1 - (2*(q[2]**2 + q[3]**2)))
    #o_with_cross = math.atan2(2*((q[0]*q[1]) + (q[2]*q[3])), 1 - (2*(q[1]**2 + q[2]**2)))
    #theta = math.asin(2*((q[0]*q[2])-(q[3]*q[1])))
    #return theta
    # x, y, z = euler_from_quaternion(q[0],
    #  q[1], q[2], q[3])
    return Quaternion(q[3], q[0], q[1], q[2]).radians


def optitrack_callback(bodies, markers, timing):
    global optitrack_id, optitrack_data, last_time
    for body in bodies:
        if body.id_ != optitrack_id:
            continue
        
        #print(body)
        #print(quaternion_to_angle(body.orientation))
        #quaternionTest.printQuaternionInfo(body.orientation)
        now = current_milli_time()

        # print('\t'.join([str(x) for x in body.orientation]), end='\t')
        # print((now-last_time))
        last_time = now
        optitrack_data = body


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
        self.ultrasonic_data = values

    def _optitrack_callback(self, bodies, markers, timing):
        print(f'Got {len(bodies)} bodies')

    async def start(self):
        self.arduino = ArduinoManager('COM10', self._ultrasonic_callback)
        self.optitrack = OptitrackManager(self._optitrack_callback)
        self.telemetry = TelemetryManager(9090)

        await asyncio.gather(
            self.arduino.start(),
            #self.optitrack.start(),
            self.telemetry.start(),
            self._loop()
        )

    async def _loop(self):
        await asyncio.sleep(1.0)
        while True:
            robot.set_position((0, 0), 0)
            robot.set_ultrasonic(self.ultrasonic_values)

            v, w = robot.get_velocity(self.telemetry.frame)

            self.arduino.send_speed(v, w)
            self.telemetry.send_frame()

            await asyncio.sleep(0.5)

    def kill(self):
        self.arduino.send_kill()


if __name__ == '__main__':
    coordinator = Coordinator()
    try:
        asyncio.get_event_loop().run_until_complete(coordinator.start())
    except KeyboardInterrupt:
        coordinator.kill()
        print('Killed by interrupt')
