#!/usr/bin/env python3
import math
from itertools import permutations
from pyquaternion import Quaternion

testValues = (-0.33, 0.26, 0.61, -0.67)

lastKnownGoodValue = 0

def euler_from_quaternion1(w, x, y, z):
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

def euler_from_quaternion2(w, x, y, z):
    """
    Converts quaternion to bank, pitch, heading using slightly different formulae
    """
    p = math.asin(-2 * (y*z + w*x))
    h = math.atan2(2*x*y - 2*w*y, 1 - 2*(x**2) - 2*(y**2)) if math.cos(p) != 0 else math.atan2(-x*z - w*y, 0.5 - (y**2) - (z**2))
    b = math.atan2(x*y - w*z, 0.5 - x**2 - z**2) if math.cos(p) != 0 else 0
    return (b, p, h)

def euler_from_quaternion3(w, x, y, z, euler_form='zxy', degrees=True):
    from scipy.spatial.transform import Rotation as R
    q = R.from_quat([x, y, z, w])
    return q.as_euler(euler_form, degrees=degrees)

def headingFromQuaternion(q):
    """Returns a 2D heading expressed as a radian from pi to -pi"""
    errorMargins = 3.0
    
    y1 = -euler_from_quaternion3(*q, 'yzx', degrees=False)[0]
    y2 = euler_from_quaternion3(*q, 'xzy', degrees=False)[2]

    # Tried bad: yxz
    # Tried decent: zxy[2] (jumps *sometimes*, fairly stable)

    if (abs(y1 - y2) < errorMargins or abs(-y1 - y2) < errorMargins):
        return (lastKnownGoodValue := y1)
    else:
        return lastKnownGoodValue

def printQuaternionInfo(q):
    errorMargins = 3.0
    qObj = Quaternion(q[0], q[1], q[2], q[3])
    # Euler 'XYZ': {list(map(lambda t: t * 180 / math.pi, euler_from_quaternion_chris(*q)))}, Euler 'BPH': {list(map(lambda t: t * 180 / math.pi, euler_from_quaternion(*q)))}
    # print(f"Angle: {qObj.degrees};  Axis: {qObj.axis}; Euler 'zxy': {euler_from_quaternion3(*q)} Input: {list(map(str, q))}")
    # y1 = -euler_from_quaternion3(*q, 'yzx')[0]
    # y2 = euler_from_quaternion3(*q, 'xzy')[2]
    # print(f"Euler 'yzx': {y1}; Euler 'xzy': {y2}") # Input: {list(map(str, q))}")
    print(headingFromQuaternion(q), q)
    

if __name__ == "__main__":
    for i in permutations(testValues):
        printQuaternionInfo(i)
