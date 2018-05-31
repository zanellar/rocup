#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf

from rocup.proxy.command_proxy import CommandProxyClient
from rocup.proxy.proxy_message import SimpleMessage
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

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("supervisor_proxy_test_2")
node.setupParameter("hz", 60)
node.setHz(node.getParameter("hz"))

pose_pub = node.createPublisher("/robot_pose", Pose)

counter = 0
active_command = None


def sendNewMessage():
    global counter
    command = msgs[counter % len(msgs)]
    counter += 1
    message = SimpleMessage(
        receiver="comau_smart_six_supervisor", command="gototf")
    message.setData("tf_name", command)
    message.setData("tool_name", "camera")
    print("Sending", command)
    active_command = command_proxy_client.sendCommand(message.toString())


def done_example(command):
    global active_command
    print("Response:", str(command))
    print(json.dumps(command.response_data))
    sendNewMessage()


command_proxy_client = CommandProxyClient("comau_smart_six_supervisor")
command_proxy_client.registerDoneCallback(done_example)


def addScanPose(array, pose_name, use_skew=False):
    array.append(pose_name)
    if use_skew:
        array.append(pose_name + "_s1")
        array.append(pose_name + "_s2")


msgs = []

use_skew = True


addScanPose(msgs, "scan_tf_8_b")
addScanPose(msgs, "scan_tf_15_b")

addScanPose(msgs, "scan_tf_14_b")
addScanPose(msgs, "scan_tf_13_b")
addScanPose(msgs, "scan_tf_4_b", use_skew=False)
addScanPose(msgs, "scan_tf_11_b")
addScanPose(msgs, "scan_tf_10_b")
addScanPose(msgs, "scan_tf_11_b", use_skew=False)
addScanPose(msgs, "scan_tf_4_b", use_skew=False)
addScanPose(msgs, "scan_tf_13_b", use_skew=False)
addScanPose(msgs, "scan_tf_14_b", use_skew=False)
addScanPose(msgs, "scan_tf_15_b", use_skew=False)
addScanPose(msgs, "scan_tf_8_b", use_skew=False)

print msgs

sendNewMessage()
while node.isActive():

    wrist = node.retrieveTransform(
        "comau_smart_six/link6",
        "comau_smart_six/base_link",
        -1
    )
    if wrist:
        pose = transformations.KDLToPose(wrist)
        pose_pub.publish(pose)
    node.tick()
