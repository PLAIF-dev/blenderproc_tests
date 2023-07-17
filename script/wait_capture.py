'''sample code which generates a simple image'''

import random
import time
import sys
import datetime
import blenderproc as bproc
import bpy
import cv2
import numpy as np

class WaitCapture():
    '''wait capture docstr'''

    def __init__(self, object_folder_path, result_folder_path):
        self.generate_images(object_folder_path, result_folder_path)

    def generate_images(self, object_folder_path, result_folder_path):
        '''generate images docstr'''
        _object_folder_path = object_folder_path
        bproc.init()
        self.set_light(self.get_light())
        bproc.loader.load_obj('/home/plaif/SyntheticGenerator/TEST3/Object/Can/Can.obj')
        self.get_and_set_camera()
        rendered_data = bproc.renderer.render()
        color = rendered_data["colors"][0]

        # rgb tp bgr
        color[..., :3] = color[..., :3][..., ::-1].astype(np.uint8)

        # save or send
        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y%m%d_%H%M%S")
        result_file = result_folder_path + f"/result_{formatted_datetime}.png"
        print(f"[Wait_capture] Result image is generated at [{result_file}]")

        cv2.imwrite(result_file,color)
        bproc.utility.reset_keyframes()

    def get_light(self):
        '''get bproc light'''
        # define a light and set its location and energy level
        light = bproc.types.Light()
        light.set_type("POINT")
        return light

    def set_light(self, light):
        '''set bproc light at fixed point'''
        _x = random.randint(-1500, 1500)/1000
        _y = random.randint(-1500, 1500)/1000
        _z = 2000/1000
        energy =  random.randint(100, 500)
        light.set_location([_x, _y, _z])
        light.set_energy(energy)
        return light

    def get_and_set_camera(self):
        '''docstr'''
        cam = bproc.camera
        cam.set_resolution(640, 480)
        focal_length = 35
        cam.set_intrinsics_from_blender_params(lens=focal_length, lens_unit='MILLIMETERS')

        # focus_point = bproc.object.create_empty("Camera Focus Point")
        # focus_point.set_location([0, 0, 0])
        # cam.add_depth_of_field(focus_point, fstop_value=0.25,focal_distance=0.8)

        _x = 0
        _y = 0
        _z =  random.uniform(600, 1400)/1000
        # yaw = np.pi/ random.uniform(-0.25, 0.25)
        yaw = random.uniform(-0.25, 0.25)
        roll = random.uniform(-0.05, 0.05)
        pitch = random.uniform(-0.05, 0.05)
        cam_pose = bproc.math.build_transformation_mat([_x, _y, _z], [roll, pitch, yaw])
        cam.add_camera_pose(cam_pose)

        return cam

if __name__ == '__main__':
    OBJECT_PATH = sys.argv[1]
    RESULT_PATH = sys.argv[2]
    WaitCapture(OBJECT_PATH, RESULT_PATH)
