#!/usr/bin/env python3
import time
import math

import natnet
import transformations as tf

optitrack_id = 25
client = natnet.Client.connect()

def get_orientation(bodies, markers, timing):
    global optitrack_id
    # Get the RigidBody for our robot
    ourBody = [b for b in bodies if b.id_ == optitrack_id][0]
    return (ourBody.orientation[1], ourBody.orientation[2], ourBody.orientation[3], ourBody.orientation[0])

def print_euler(orient):
    #print(orient)
    euler = tf.euler_from_quaternion(orient)
    angle = euler[1]
    #if angle > 0:
    if(abs(euler[2]) < 2 or angle < 0):
        angle += ((math.pi / 2) - angle) * 2
    if(euler[1] < 0 and abs(euler[2]) > 2):
        angle += ((math.pi / 2) - abs(euler[1])) * 2
    print(f'{angle}\t\t{euler[0]}\t{euler[1]}\t{euler[2]}')
    #print(euler)

def print_quaternion_timing():
    def print_quat(r,m,t):
        quad = get_orientation(r,m,t)
        cur_time = math.trunc(time.time() *1000)
        print_euler(quad)
        #print(quad, cur_time)
    client.set_callback(print_quat)
    try:
        client.spin()
    except KeyboardInterrupt:
        print("End Stream")

if __name__ == "__main__":
    print_quaternion_timing()