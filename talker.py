#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = Twist(Vector3(0,0,0), Vector3(0,0,-3.0))
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
