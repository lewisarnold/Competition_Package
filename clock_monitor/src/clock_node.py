#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Clock_Node():
    def setup(self):

        rospy.init_node('clock_node')

        #self.subscriber = rospy.Subscriber('/clock', String, self.callback)

        while(True):
            print(rospy.get_time())




if __name__ == '__main__':
    hey = Clock_Node()
    hey.setup()
