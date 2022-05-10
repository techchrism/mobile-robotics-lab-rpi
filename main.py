from OptitrackManager import OptitrackManager
from ArduinoManager import ArduinoManager
from RobotBase import RobotBase
import time
import threading


optitrack_data = None
optitrack_id = 20
ultrasonic_data = None
robot = RobotBase()


def optitrack_callback(bodies, markers, timing):
    global optitrack_id, optitrack_data
    for body in bodies:
        if body.id_ != optitrack_id:
            continue
        
        optitrack_data = body


def ultrasonic_callback(values):
    global ultrasonic_data
    ultrasonic_data = values


def async_loop():
    global optitrack_data, ultrasonic_data, arduino

    time.sleep(1)
    while True:
        robot.set_position((0, 0), 0)
        robot.set_ultrasonic(ultrasonic_data)
        v, w = robot.get_velocity()
        arduino.send_speed(v, w)
        time.sleep(0.1)


test = OptitrackManager(optitrack_callback)
arduino = ArduinoManager('/dev/ttyACM0', ultrasonic_callback)
arduino.start()

test.start()
t = threading.Thread(target=async_loop)
t.start()
