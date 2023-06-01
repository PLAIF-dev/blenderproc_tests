#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sensor_msgs
from PIL import Image as I
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Header

def rgb_to_msg(img):
    # Create a ROS1 Image message
    ros_msg = Image()
    ros_msg.header = Header(
        stamp=rospy.Time.now()
    )  # Set the header with current time
    ros_msg.height = img.shape[0]  # Set image height
    ros_msg.width = img.shape[1]  # Set image width
    ros_msg.encoding = (
        "rgb8"  # Set encoding as 32-bit floating point (single channel)
    )
    ros_msg.is_bigendian = False  # Set endianness
    ros_msg.step = ros_msg.width * 3  # Set step size (4 bytes per float)

    # Convert NumPy array to bytes and set image data
    ros_msg.data = img.tobytes()

    return ros_msg

class Blenderproc_tests():
    def __init__(self):
        rospy.init_node('Blenderproc_tests_node')
        rospy.Subscriber('create_new_image', String, self.callback_create)
        rospy.loginfo('[Blenderproc_tests_node] started')
        try:
            rospy.spin()
        except:
            rospy.signal_shutdown('Blenderproc_tests_node is shutdown')


    def callback_create(self, msg):
        rospy.loginfo('[Blenderproc_tests_node] callback_create called')
        cmd = "blenderproc run /home/yhpark/catkin_ws/src/blenderproc_tests/script/wait_capture.py"
        returned_value = os.system(cmd)  # returns the exit code in unix
        print('returned value:', returned_value)
        self.send_image()

    def send_image(self):
        img_path = '/home/yhpark/catkin_ws/src/blenderproc_tests/script/result.png'
        img = I.open(img_path)

        img = np.asarray(img)
        img_msg = rgb_to_msg(img)

        img_publisher = rospy.Publisher('new_image_topic', Image, queue_size=1)
        img_publisher.publish(img_msg)


if __name__ == '__main__':
    Blenderproc_tests()