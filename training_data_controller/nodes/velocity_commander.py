#!/usr/bin/env python

#Code shell copied from ROS Tutorial on listenr node and subscriber node

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def velocity_commander():
    pub = rospy.Publisher('R1/cmd_vel', Twist, queue_size=1)
    rospy.init_node('velocity_commander')

    rate = rospy.Rate(2) # 2hz

    move_command = Twist()
    move_command.linear.x = 0.0
    move_command.angular.z = 0.5

    while not rospy.is_shutdown():
        pub.publish(move_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_commander()
    except rospy.ROSInterruptException:
        pass
