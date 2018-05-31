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
from tf_matricies import getTfMatrix


#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("matrix_tf_sequencer")
node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))
node.setupParameter("scan", "default")
scan_name = node.getParameter("scan")
node.setupParameter("robot", "bonmetc60")
robot_name = node.getParameter("robot")

moving_pub = node.createPublisher("~moving_flag", Int32)
pose_pub = node.createPublisher("~tool_pose", Pose)

counter = 0
active_command = None

moving_flag = -1

number_of_frames = 0.0
done_frames = 0.0


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ SUPERVISOR COMUNIOCATION ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


def goToTf(tf_name, tool="camera"):
    #⬢⬢⬢⬢⬢➤ Go To Tf Helper Function
    global command_proxy_client, moving_flag
    message = SimpleMessage(
        receiver="{}_supervisor".format(robot_name), command="jumptotf")
    message.setData("tf_name", tf_name)
    message.setData("tool_name", tool)
    # print("Sending", message.toString())
    active_command = command_proxy_client.sendCommand(message.toString())
    moving_flag = 1


def sendNewMessage():
    #⬢⬢⬢⬢⬢➤ Send New Message to Proxy
    global counter, matrix_tf
    frame_name, _ = matrix_tf.pickNextFrame()
    goToTf(frame_name)


def supervisor_done_callback(command):
    #⬢⬢⬢⬢⬢➤ Action Callback
    global moving_flag, matrix_tf, done_frames
    moving_flag = 0
    done_frames += 1.0
    print("{0:.2f} %".format(done_frames / number_of_frames * 100.0))
    sendNewMessage()


command_proxy_client = CommandProxyClient("{}_supervisor".format(robot_name))
command_proxy_client.registerDoneCallback(supervisor_done_callback)


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ TF MATRIX ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

#⬢⬢⬢⬢⬢➤ Retrive Chessboard TF
base_frame = None
while base_frame is None:
    base_frame = node.retrieveTransform("tf_storage_scan_center", "/{}/base_link".format(robot_name), -1)

print("Chessboard TF FOUND!!!!!!!!")

matrix_cube = getTfMatrix(robot_name, scan_name, base_frame)
matrix_tf = matrix_cube

number_of_frames = len(matrix_cube.frames_names)

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

sendNewMessage()

moving_msg = Int32()

os.system('clear')
while node.isActive():
    moving_msg.data = moving_flag
    moving_pub.publish(moving_msg)

    tool_pose = Pose()

    tool_frame = node.retrieveTransform("/{}/tool".format(robot_name), "/{}/base_link".format(robot_name), -1)
    if tool_frame is not None:
        tool_pose = transformations.KDLToPose(tool_frame)

    pose_pub.publish(tool_pose)
    node.tick()
