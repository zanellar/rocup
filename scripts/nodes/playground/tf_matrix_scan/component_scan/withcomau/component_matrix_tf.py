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
import aruco
import rospkg
import numpy as np
import math
import json
from rocup.utils.tfmatrix import TfMatrixCube, TfMatrixHyperCube, TfMatrixSphere

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("matrix_tf_generator")
node.setupParameter("hz", 1)
node.setHz(node.getParameter("hz"))

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ TF MATRIX ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

node.setupParameter("offset_cube", [0.0, 0, 0])

nX = 3
nY = 2  # 4
nZ = 3

dX = 0.2
dY = 0.15  # 0.1
dZ = -0.05

nRoll = 1
nPitch = 1
nYaw = 4  # 8

dRoll = 0
dPitch = 0
dYaw = math.pi / 2  # /4

robot_name = "bonmetc60"  # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

#⬢⬢⬢⬢⬢➤ Retrive Chessboard TF
base_frame = None
while base_frame is None:
    base_frame = node.retrieveTransform("tf_storage_scan_center", "/{}/base_link".format(robot_name), -1)

print("Chessboard TF FOUND!!!!!!!!")

#⬢⬢⬢⬢⬢➤ Build the TF Matrix
offset_cube = node.getParameter("offset_cube")

cube_center_frame = PyKDL.Frame()
cube_center_frame.M = base_frame.M
cx = dX * float(nX - 1) / 2.0
cy = - dY * float(nY - 1) / 2.0
cube_center_frame.p = base_frame.p + PyKDL.Vector(cx + offset_cube[0], cy + offset_cube[1], 0.0 + offset_cube[2])    # coords first tf wrt base-tf
matrix_cube = TfMatrixHyperCube(
    base=cube_center_frame,
    size_p=np.array([nX, nY, nZ]),
    size_m_p=np.array([-dX, -dY, dZ]),
    size_r=np.array([nRoll, nPitch, nYaw]),
    size_m_r=np.array([dRoll, dPitch, dYaw])
)
matrix_cube.order(mode="random")

print matrix_cube.frames_names
while node.isActive():

    for name, frame in matrix_cube.frames_map.iteritems():
        node.broadcastTransform(frame, name, "/{}/base_link".format(robot_name), node.getCurrentTime())

    node.tick()
