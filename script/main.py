#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Blenderproc_tests():
    def __init__(self):
        rospy.init_node('Blenderproc_tests_node')
        # subscriber
        rospy.Subscriber('create_new_image', String, self.callback_create)
        rospy.loginfo('[Blenderproc_tests_node] started')
        try:
            rospy.spin()
        except:
            rospy.signal_shutdown('Blenderproc_tests_node is shutdown')

    def callback_create(self, msg):
        rospy.loginfo('[Blenderproc_tests_node] callback_vision_onoff_cmd starts..!')

if __name__ == '__main__':
    Blenderproc_tests()