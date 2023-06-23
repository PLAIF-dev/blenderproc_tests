import blenderproc as bproc
import random
import bpy
import cv2
import numpy as np
import time


def get_light():
    # define a light and set its location and energy level
    light = bproc.types.Light()
    light.set_type("POINT")
    return light

def set_light(light):
    x = random.randint(-1500, 1500)/1000
    y = random.randint(-1500, 1500)/1000
    z = 2000/1000
    energy =  random.randint(100, 500)
    light.set_location([x, y, z])
    light.set_energy(energy)
    return light

def get_and_set_camera():
    cam = bproc.camera
    cam.set_resolution(640, 480)
    focal_length = 35
    cam.set_intrinsics_from_blender_params(lens=focal_length, lens_unit='MILLIMETERS')

    # focus_point = bproc.object.create_empty("Camera Focus Point")
    # focus_point.set_location([0, 0, 0])
    # cam.add_depth_of_field(focus_point, fstop_value=0.25,focal_distance=0.8)

    x = 0
    y = 0
    z =  random.uniform(600, 1400)/1000
    # yaw = np.pi/ random.uniform(-0.25, 0.25)
    yaw = random.uniform(-0.25, 0.25)
    roll = random.uniform(-0.05, 0.05)
    pitch = random.uniform(-0.05, 0.05)
    cam_pose = bproc.math.build_transformation_mat([x, y, z], [roll, pitch, yaw])
    cam.add_camera_pose(cam_pose)

    return cam

bproc.init()
set_light(get_light())

obj_list = ['/home/plaif/catkin_ws/src/blenderproc_tests/script/002_master_chef_can_ycb/textured.obj',
            '/home/plaif/catkin_ws/src/blenderproc_tests/script/003_cracker_box_ycb/textured.obj',
            '/home/plaif/catkin_ws/src/blenderproc_tests/script/006_mustard_bottle_ycb/textured.obj',
            '/home/plaif/catkin_ws/src/blenderproc_tests/script/008_pudding_box_ycb/textured.obj',
            '/home/plaif/catkin_ws/src/blenderproc_tests/script/009_gelatin_box_ycb/textured.obj']
obj = bproc.loader.load_obj(random.choice(obj_list))

get_and_set_camera()
rendered_data = bproc.renderer.render()
color = rendered_data["colors"][0]

# rgb tp bgr
color[..., :3] = color[..., :3][..., ::-1].astype(np.uint8)

# save or send
cv2.imwrite('/home/plaif/catkin_ws/src/blenderproc_tests/script/result.png',color)

bproc.utility.reset_keyframes()
