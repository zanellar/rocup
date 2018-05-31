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

from superros.comm import RosNode
from superros.logger import Logger
from rocup.proxy.command_proxy import CommandProxyClient
from rocup.proxy.command_proxy import CommandProxyClient
from rocup.proxy.proxy_message import SimpleMessage
import superros.transformations as transformations
import message_filters
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import numpy as np
import math
import json
from rocup.utils.tfmatrix import TfMatrixCube, TfMatrixHyperCube, TfMatrixSphere
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import threading


def getTfMatrix(robot_name, scan_name, base_frame):
    if scan_name == "default":

        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        if robot_name == "bonmetc60":
            offset_cube = [0, 0, 0]

            nX = 1
            nY = 1  # 4
            nZ = 3

            dX = 0.4
            dY = 0.15  # 0.1
            dZ = -0.05

            nRoll = 1
            nPitch = 1
            nYaw = 8

            dRoll = 0
            dPitch = 0
            dYaw = math.pi / 4

            ##########################################################################

            cube_center_frame = PyKDL.Frame()
            cube_center_frame.M = base_frame.M
            cx = dX * float(nX - 1) / 2.0
            cy = - dY * float(nY - 1) / 2.0
            cube_center_frame.p = base_frame.p  # + PyKDL.Vector(cy + offset_cube[0],
            #   -cx + offset_cube[1],
            #   0.0 + offset_cube[2])    # coords first tf wrt base-tf
            matrix_cube = TfMatrixHyperCube(
                base=cube_center_frame,
                size_p=np.array([nX, nY, nZ]),
                size_m_p=np.array([-dX, -dY, dZ]),
                size_r=np.array([nRoll, nPitch, nYaw]),
                size_m_r=np.array([dRoll, dPitch, dYaw])
            )
            # matrix_cube.order(mode="xyzrpy")

            return matrix_cube

        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        elif robot_name == "comau_smart_six":
            offset_cube = [0, 0, 0]

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

            ##########################################################################

            cube_center_frame = PyKDL.Frame()
            cube_center_frame.M = base_frame.M
            cx = dX * float(nX - 1) / 2.0
            cy = - dY * float(nY - 1) / 2.0
            cube_center_frame.p = base_frame.p + PyKDL.Vector(cx + offset_cube[0],
                                                              cy + offset_cube[1],
                                                              0.0 + offset_cube[2])    # coords first tf wrt base-tf
            matrix_cube = TfMatrixHyperCube(
                base=cube_center_frame,
                size_p=np.array([nX, nY, nZ]),
                size_m_p=np.array([-dX, -dY, dZ]),
                size_r=np.array([nRoll, nPitch, nYaw]),
                size_m_r=np.array([dRoll, dPitch, dYaw])
            )
            # matrix_cube.order(mode="random")

            return matrix_cube
