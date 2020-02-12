#!/usr/bin/env python
import rospy
import time      
from geometry_msgs.msg import Point
from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement   
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('navigator', anonymous=True)
def get_direction_msg(direction):
    if direction == 'forward':
        return Twist(
            Vector3(1.5,0,0),     # linear
            Vector3(0,0,0)      # angular
        )
    elif direction == 'backward':   
        return Twist(
            Vector3(-1.5,0,0),
            Vector3(0,0,0)
        )  
    elif direction == 'stop':
         print('stop')
         return Twist(
            Vector3(0,0,0),
            Vector3(0,0,0)
          )  
      
def publish_callback(val):
    msg = None
    print(val)

    if val.people[0].pos.x < 0.900000000000:
        print('moving backward')
        msg = get_direction_msg('backward')
    elif val.people[0].pos.x > 0.9100000000000 and val.people[0].pos.x < 1.400000000000:
        print('stop')
        msg = get_direction_msg('stop')
    else:
          print('moving Forward')
          msg = get_direction_msg('forward')
    if msg:
        #rospy.loginfo(msg)
        pub.publish(msg)

rospy.Subscriber('leg_tracker_measurements', PositionMeasurementArray, publish_callback)
rospy.spin()
