#!/usr/bin/env python
'''synthetic launcher docstr'''

import os
import signal
import time
import json
import threading
import subprocess
import rospy
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Header
import psutil

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
        self.is_generating = False
        self.subprocess_blenderproc = None
        rospy.init_node("Synthetic_rospkg_node")
        rospy.loginfo("[Synthetic_rospkg_node] started")
        rospy.Subscriber("generate_image", String, self.callback_generate_image)
        rospy.Subscriber("start_learn", String, self.callback_start_learn)
        rospy.Subscriber("break_generate", String, self.callback_break_generate)
        rospy.spin()

    def callback_generate_image(self, msg):
        '''callback of topic [generate_image]'''

        rospy.Publisher("generate_status", String, queue_size=1).publish("in progress")
        rospy.loginfo("[Synthetic_rospkg_node] callback_generate_image called")

        split_values = msg.data.split(" ") # "100 20" -> "100" & "20"
        set_count = int(split_values[0])
        iteration_count = int(split_values[1])
        total_count = set_count * iteration_count

        project_folder_path = os.path.expanduser(
            "~/SyntheticGenerator/" + self.get_current_project_name()
        )
        object_folder_path = project_folder_path + "/Object"
        result_folder_path = project_folder_path + "/Result"

        # temporarily clear result folder
        os.system("rm -rf {}".format(result_folder_path))

        # run check_image_generation as a thread
        self.is_generating = True
        checker_thread = threading.Thread(
            target=self.check_image_generation,
            args=(result_folder_path+"/color", total_count))
        checker_thread.start()

        for i in range(iteration_count):
            if self.is_generating is False:
                rospy.loginfo("[Synthetic_rospkg_node] is_generating is false in for loop")
                break
            rospy.Publisher("generate_progress", String, queue_size=1).publish(
                "{}/{}".format(i+1, iteration_count))
            rospy.Publisher("generate_status", String, queue_size=1).publish("loading")
            self.run_blenderproc(object_folder_path, result_folder_path, set_count)

        checker_thread.join()
        self.is_generating = False
        rospy.Publisher("generate_status", String, queue_size=1).publish("finished")
        rospy.loginfo("[Synthetic_rospkg_node] callback_generate_image finished")

    def check_image_generation(self, path, total_count):
        '''periodically check image generating progress and notify'''
        interval_second = 1
        file_count_prev = self.get_file_count(path)
        generated_count_prev = 0
        while self.is_generating is True:
            time.sleep(interval_second)

            if not self.subprocess_blenderproc or \
                self.is_process_running(self.subprocess_blenderproc.pid) is False:
                break

            generated_count = self.get_file_count(path) - file_count_prev
            _is_updated = generated_count > generated_count_prev
            if _is_updated is False:
                continue

            _is_finished = total_count == generated_count
            if _is_finished:
                break

            rospy.Publisher("generate_status", String, queue_size=1).publish("generating")
            rospy.Publisher("generate_rate", String, queue_size=1).publish(
                "{}/{}".format(generated_count, total_count))
            generated_count_prev = generated_count
        rospy.loginfo("[Synthetic_rospkg_node] check_image_generation finished")

    def run_blenderproc(self, object_folder_path, result_folder_path, set_count):
        '''run blenderproc'''
        _set_count = set_count
        blenderproc_filepath = os.path.expanduser(
            "~/catkin_ws/src/synthetic_rospkg/vs_synthetic_generator/generate_synthetic.py")
        self.subprocess_blenderproc = subprocess.Popen(
            ["blenderproc", "run", blenderproc_filepath, object_folder_path, result_folder_path]
        )
        self.subprocess_blenderproc.wait()

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
            "~/SyntheticGenerator/" + self.get_current_project_name() + "/weight_file")
        self.create_folder_recursive(dummy_file_path)
        with open(dummy_file_path + "/weight.pth", "w") as file:
            file.write("this is a dummy weight file :)")
        rospy.Publisher("learn_status", String, queue_size=1).publish("finished")
        rospy.loginfo("[Synthetic_rospkg_node] model file is created successfully")

    def get_current_project_name(self):
        '''Get the current project name from SG_Config.json'''
        config_file = os.path.expanduser("~/SyntheticGenerator/SG_Config.json")
        if not os.path.exists(config_file):
            rospy.loginfo(
                "[Synthetic_rospkg_node] error occurred in get_current_project_name"
            )
            return ""
        with open(config_file, "r") as file:
            json_data = json.load(file)
            return json_data.get("current_project", "")

    def create_folder_recursive(self, path):
        '''Create folders recursively'''
        if not os.path.exists(path):
            os.makedirs(path)

    def get_file_count(self, folder_path):
        ''' get file count in path '''
        try:
            file_list = os.listdir(folder_path)
            file_count = len(file_list)
            return file_count
        except OSError as _e:
            print("Error while getting file count: {}".format(_e))
            return 0

    def callback_break_generate(self, msg):
        '''terminate subprocess if recieved external signal'''
        _msg = msg
        rospy.loginfo("[Synthetic_rospkg_node] callback_break_generate called")
        if self.subprocess_blenderproc:
            os.kill(self.subprocess_blenderproc.pid, signal.SIGTERM) #or signal.SIGKILL
        self.is_generating = False

    def is_process_running(self, pid):
        ''' check the process is currently running '''
        try:
            process = psutil.Process(pid)
            return process.is_running()
        except psutil.NoSuchProcess:
            return False

if __name__ == "__main__":
    print('syn_launcher pid : ', os.getpid())
    SyntheticRospkg()
