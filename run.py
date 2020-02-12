#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('navigator', anonymous=True)


def get_direction_msg(direction):
    if direction == 'forward':
        return Twist(
            Vector3(1,0,0),     # linear
            Vector3(0,0,0)      # angular
        )
    elif direction == 'backward':
        return Twist(
            Vector3(-1,0,0),
            Vector3(0,0,0)
        )
    elif direction == 'left':
        return Twist(
            Vector3(0,0,0),
            Vector3(0,0,0.3)
        )
    elif direction == 'right':
        return Twist(
            Vector3(0,0,0),
            Vector3(0,0,-0.3)
        )

def publish_callback(faceloc):
    msg = None
    print(faceloc)

    if faceloc.data > 55:
        print('moving right')
        msg = get_direction_msg('right')
    elif faceloc.data < 45:
        print('moving left')
        msg = get_direction_msg('left')

    if msg:
        #rospy.loginfo(msg)
        pub.publish(msg)

rospy.Subscriber('faceloc', Int8, publish_callback)
rospy.spin()
