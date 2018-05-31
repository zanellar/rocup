#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf

from rocup.proxy.command_proxy import CommandProxyClient
from superros.comm import RosNode
import superros.transformations as transformations
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import rospkg
import numpy as np
import math
import json
from rocup.utils.tfmatrix import TfMatrixCube, TfMatrixHyperCube, TfMatrixSphere


from tf_matricies import getTfMatrix

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("matrix_tf_generator")
node.setupParameter("hz", 1)
node.setHz(node.getParameter("hz"))
node.setupParameter("scan", "default")
scan_name = node.getParameter("scan")
node.setupParameter("robot", "bonmetc60")
robot_name = node.getParameter("robot")

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ TF MATRIX ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


#⬢⬢⬢⬢⬢➤ Retrive Chessboard TF
base_frame = None
while base_frame is None:
    base_frame = node.retrieveTransform("tf_storage_scan_center", "/{}/base_link".format(robot_name), -1)

print("Chessboard TF FOUND!!!!!!!!")

matrix_cube = getTfMatrix(robot_name, scan_name, base_frame)


print matrix_cube.frames_names
while node.isActive():

    for name, frame in matrix_cube.frames_map.iteritems():
        node.broadcastTransform(frame, name, "/{}/base_link".format(robot_name), node.getCurrentTime())

    node.tick()
