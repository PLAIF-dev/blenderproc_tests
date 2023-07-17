#!/usr/bin/env python
'''synthetic launcher docstr'''

import os
import time
import json
import rospy
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Header

def rgb_to_msg(img):
    '''convert rgb image to msg'''

    # Create a ROS1 Image message
    ros_msg = Image()
    ros_msg.header = Header(stamp=rospy.Time.now())  # Set the header with current time
    ros_msg.height = img.shape[0]  # Set image height
    ros_msg.width = img.shape[1]  # Set image width
    ros_msg.encoding = "rgb8"  # Set encoding as 32-bit floating point (single channel)
    ros_msg.is_bigendian = False  # Set endianness
    ros_msg.step = ros_msg.width * 3  # Set step size (4 bytes per float)

    # Convert NumPy array to bytes and set image data
    ros_msg.data = img.tobytes()

    return ros_msg


class SyntheticRospkg:
    '''main node of synthetic generator'''

    def __init__(self):
        rospy.init_node("Synthetic_rospkg_node")
        rospy.loginfo("[Synthetic_rospkg_node] started")
        rospy.Subscriber("generate_image", String, self.callback_generate_image)
        rospy.Subscriber("start_learn", String, self.callback_start_learn)
        rospy.spin()

    def callback_generate_image(self, msg):
        '''callback of topic [generate_image]'''

        rospy.Publisher("generate_status", String, queue_size=1).publish("in progress")
        rospy.loginfo("[Synthetic_rospkg_node] callback_generate_image called")
        count = 5 if msg.data == "" else int(msg.data)
        project_folder_path = os.path.expanduser(
            "~/SyntheticGenerator/" + self.get_current_project_name()
        )
        object_folder_path = project_folder_path + "/Object"
        result_folder_path = project_folder_path + "/Result"
        for i in range(count):
            current_progress_str = f"{i+1}/{count}"
            _pb = rospy.Publisher("generate_progress", String, queue_size=1)
            _pb.publish(current_progress_str)
            rospy.loginfo(f"#{i} : cmd")
            entry_file = os.path.expanduser(
                "~/catkin_ws/src/synthetic_rospkg/vs_synthetic_generator/generate_synthetic.py"
            )
            cmd = f"blenderproc run {entry_file} {object_folder_path} {result_folder_path}"
            os.system(cmd)  # returns the exit code in unix
        rospy.Publisher("generate_status", String, queue_size=1).publish("finished")

    def callback_start_learn(self, msg):
        '''callback of topic [start_learn]'''
        _msg = msg
        rospy.Publisher("learn_status", String, queue_size=1).publish("in progress")
        rospy.loginfo("[Synthetic_rospkg_node] callback_start_learn called")
        for i in range(5):
            _pb = rospy.Publisher("learn_progress", String, queue_size=1)
            _pb.publish(str((i + 1) * 20))
            time.sleep(1)

        # create dummy model file
        dummy_file_path = os.path.expanduser(
            "~/SyntheticGenerator/" + self.get_current_project_name() + "/weight_file"
        )
        self.create_folder_recursive(dummy_file_path)
        with open(dummy_file_path + "/weight.pth", "w", encoding="utf-8") as file:
            file.write("this is a dummy weight file :)")
        rospy.Publisher("learn_status", String, queue_size=1).publish("finished")
        rospy.loginfo("[Synthetic_rospkg_node] model file is created successfully")

    def get_current_project_name(self):
        '''callback of topic [generate_image]'''
        config_file = os.path.expanduser("~/SyntheticGenerator/SG_Config.json")
        if not os.path.exists(config_file):
            rospy.loginfo(
                "[Synthetic_rospkg_node] error occurred in get_current_project_name"
            )
            return ""
        with open(config_file, "r", encoding="utf-8") as file:
            json_data = json.load(file)
            return json_data.get("current_project", "")

    def create_folder_recursive(self, path):
        '''callback of topic [generate_image]'''
        if not os.path.exists(path):
            os.makedirs(path)


if __name__ == "__main__":
    SyntheticRospkg()
