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
import glob
import collections
import shutil

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("topic_export_filtering")

node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))
image_path = node.setupParameter("image_path", "")
pose_file = node.setupParameter("pose_file", "")
signal_file = node.setupParameter("signal_file", "")
output_path = node.setupParameter("output_path", "/tmp/topic_export_filtering/")
zfill = node.setupParameter("zfill", 5)

if not os.path.exists(output_path):
    os.makedirs(output_path)

images = sorted(glob.glob(os.path.join(image_path, "*.*")))

pose_file = np.loadtxt(pose_file)
signal_file = np.loadtxt(signal_file)


current_flag = 0
counter = -1
data_map = collections.OrderedDict()
for i in range(0, len(images)):

    if signal_file[i] != current_flag:
        current_flag = signal_file[i]
        if signal_file[i] == 0.0:
            counter += 1

    if counter >= 0:
        if signal_file[i] == 0.0:
            data_map[counter] = (images[i], pose_file[i, :])


for k, v in data_map.iteritems():
    print(k)
    _, ext = os.path.splitext(v[0])
    name = "{}".format(str(k).zfill(zfill))
    image_path = os.path.join(output_path, name + ext)
    pose_path = os.path.join(output_path, name + ".txt")
    shutil.copy(v[0], image_path)
    np.savetxt(pose_path, v[1].reshape(1, 7))
    print(k, image_path, pose_path)

print(len(data_map))
