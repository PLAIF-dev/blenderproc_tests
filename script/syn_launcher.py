#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sensor_msgs
from PIL import Image as I
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Header
import time
import json

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

class Synthetic_rospkg():
    def __init__(self):
        rospy.init_node('Synthetic_rospkg_node')
        rospy.loginfo('[Synthetic_rospkg_node] started')
        rospy.Subscriber('generate_image', String, self.callback_generate_image)
        rospy.Subscriber('start_learn', String, self.callback_start_learn)
        try:
            rospy.spin()
        except:
            rospy.signal_shutdown('Synthetic_rospkg_node is shutdown')

    def callback_generate_image(self, msg):
        rospy.Publisher('generate_status', String, queue_size=1).publish('in progress')
        rospy.loginfo('[Synthetic_rospkg_node] callback_generate_image called')
        count = 10
        project_folder_path = os.path.expanduser('~/SyntheticGenerator/' + self.get_current_project_name()) 
        object_folder_path = project_folder_path + '/Object'
        result_folder_path = project_folder_path + '/Result'
        for i in range(count):
            current_progress_str = "{}/{}".format(i+1, count)
            pb = rospy.Publisher('generate_progress', String, queue_size=1)
            pb.publish(current_progress_str)
            rospy.loginfo('#{} : cmd'.format(i))
            entry_file = "/home/plaif/catkin_ws/src/synthetic_rospkg/script/wait_capture.py"
            cmd = "blenderproc run {} {} {}".format(
                entry_file, object_folder_path, result_folder_path)
            returned_value = os.system(cmd)  # returns the exit code in unix
        rospy.Publisher('generate_status', String, queue_size=1).publish('finished')

    def callback_start_learn(self, msg):
        rospy.Publisher('learn_status', String, queue_size=1).publish('in progress')
        rospy.loginfo('[Synthetic_rospkg_node] callback_start_learn called')
        for i in range(5):
            pb = rospy.Publisher('learn_progress', String, queue_size=1)
            pb.publish(str((i+1)*20))
            time.sleep(1)
            
        # create dummy model file
        dummy_file_path = os.path.expanduser('~/SyntheticGenerator/' + self.get_current_project_name() + "/weight_file")
        self.create_folder_recursive(dummy_file_path)
        with open(dummy_file_path + '/weight.pth', "w") as file:
            file.write('this is a dummy weight file :)')
        rospy.Publisher('learn_status', String, queue_size=1).publish('finished')
        rospy.loginfo('[Synthetic_rospkg_node] model file is created successfully')

    def send_image(self):
        img_path = '/home/yhpark/catkin_ws/src/synthetic_rospkg/script/result.png'
        img = I.open(img_path)
        img = np.asarray(img)
        img_msg = rgb_to_msg(img)
        img_publisher = rospy.Publisher('new_image_topic', Image, queue_size=1)
        img_publisher.publish(img_msg)

    def get_current_project_name(self):
        config_file = os.path.expanduser('~/SyntheticGenerator/SG_Config.json')
        if not os.path.exists(config_file):
            rospy.loginfo('[Synthetic_rospkg_node] error occurred in get_current_project_name')
            return ''
        with open(config_file, "r") as file:
            json_data = json.load(file)
            return json_data.get("current_project", '')

    def create_folder_recursive(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

if __name__ == '__main__':
    Synthetic_rospkg()
