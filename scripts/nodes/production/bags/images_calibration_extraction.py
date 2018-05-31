#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
import superros.transformations as transformations
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
import numpy as np
import PyKDL

# DA SPOSTARE
import message_filters
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image, CompressedImage
from geometry_msgs.msg import Twist
import datetime
import copy
import os
import shutil

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("images_calibration_extraction")

node.setupParameter("hz", 10)
node.setHz(node.getParameter("hz"))
images_folder = node.setupParameter(
    "images_folder", "/home/daniele/Desktop/datasets/roars_2017/indust/indust_calibration/camera_rgb_image_raw_compressed")
poses_file = node.setupParameter(
    "poses_file", "/home/daniele/Desktop/datasets/roars_2017/indust/indust_calibration/tf#_comau_smart_six_link6.txt")
zero_padding = node.setupParameter("zero_padding", 5)
image_extension = node.setupParameter("image_extension", "jpg")
output_folder = node.setupParameter("output_folder", "/tmp/images_output")
subset_percentage = node.setupParameter("subset_percentage", 0.022)

poses = np.loadtxt(poses_file)
images = []
for i in range(0, poses.shape[0]):
    image_filename = os.path.join(
        images_folder,
        'rgb_{}.{}'.format(
            str(i).zfill(zero_padding),
            image_extension)
    )
    if os.path.exists(image_filename):
        images.append(image_filename)

print(len(poses), len(images))

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

full_size = len(images)
new_size = int(full_size * subset_percentage)
slots = int(full_size / new_size)
new_images = []
new_poses = np.array([])
for i in range(0, new_size):
    pick_index = i * slots
    pose = poses[pick_index, :]
    image = images[pick_index]

    try:
        new_poses = np.vstack((new_poses, pose))
    except:
        new_poses = np.array([pose])

    new_images.append(image)

for i in range(0, len(new_images)):
    image = new_images[i]
    new_name = 'rgb_{}.{}'.format(
        str(i).zfill(zero_padding),
        image_extension)
    shutil.copy(image, os.path.join(output_folder, new_name))

    pose = new_poses[i, :]

    new_name = 'pose_{}.txt'.format(
        str(i).zfill(zero_padding))

    np.savetxt(os.path.join(output_folder, new_name), pose.reshape(1, 7))

print(new_poses)
print(len(new_images), len(new_poses))
