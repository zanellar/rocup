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
    if counter >= len(msgs):
        return
    command = msgs[counter % len(msgs)]
    counter += 1
    message = SimpleMessage(
        receiver="comau_smart_six_supervisor", command="gotoshape")
    message.setData("shape_name", command)
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

#addScanPose(msgs, "shape_scan_tf_3")
addScanPose(msgs, "shape_scan_tf_4")
addScanPose(msgs, "shape_scan_tf_5")
addScanPose(msgs, "shape_scan_tf_6")
addScanPose(msgs, "shape_scan_tf_7")
addScanPose(msgs, "shape_scan_tf_8")
addScanPose(msgs, "shape_scan_tf_13")
addScanPose(msgs, "shape_scan_tf_14")
addScanPose(msgs, "shape_scan_tf_15")
addScanPose(msgs, "shape_scan_tf_0")
addScanPose(msgs, "shape_scan_tf_1")
addScanPose(msgs, "shape_extra_s1")
addScanPose(msgs, "shape_extra_s2")
addScanPose(msgs, "shape_extra_s3")
addScanPose(msgs, "shape_extra_s4")
addScanPose(msgs, "shape_extra_s5")
addScanPose(msgs, "shape_extra_s6")

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
