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
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import aruco
import rospkg
import numpy as np
import math
import json

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("command_proxy_example_client")
node.setupParameter("hz", 1)
node.setHz(node.getParameter("hz"))

counter = 0
active_command = None


def sendNewMessage():
    global counter, active_command
    counter += 1
    command = "Garbage Command {}".format(counter)
    #⬢⬢⬢⬢⬢➤ SEND COMMAND
    print("Sending command: " + command)
    active_command = command_proxy_client.sendCommand(command)


def done_example(command):
    global active_command
    print("Response:", str(command))
    print(json.dumps(command.response_data))
    assert(command.id == active_command.id)
    sendNewMessage()


command_proxy_client = CommandProxyClient("pino")
command_proxy_client.registerDoneCallback(done_example)


sendNewMessage()
while node.isActive():

    node.tick()
