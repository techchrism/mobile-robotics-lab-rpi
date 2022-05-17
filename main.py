from OptitrackManager import OptitrackManager
from ArduinoManager import ArduinoManager
from RobotBase import RobotBase
import time
import threading
import math
from pyquaternion import Quaternion

import quaternionTest

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
        quaternionTest.printQuaternionInfo(body.orientation)
        now = current_milli_time()

        # print('\t'.join([str(x) for x in body.orientation]), end='\t')
        # print((now-last_time))
        last_time = now
        optitrack_data = body


def ultrasonic_callback(values):
    global ultrasonic_data
    values = [x or 999 for x in values]
    ultrasonic_data = values


def async_loop():
    global optitrack_data, ultrasonic_data, arduino

    time.sleep(1)
    print('Starting')
    try:
        while True:
            robot.set_position((0, 0), 0)
            robot.set_ultrasonic(ultrasonic_data)
            v, w = robot.get_velocity()
            arduino.send_speed(v, w)
            time.sleep(0.1)
    except KeyboardInterrupt:
        arduino.send_speed(0, 0)
        return


test = OptitrackManager(optitrack_callback)
arduino = ArduinoManager('/dev/ttyACM0', ultrasonic_callback)
arduino.start()

test.start()
t = threading.Thread(target=async_loop)
t.start()
