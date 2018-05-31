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
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer
from superros.comm import RosNode
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import aruco
import rospkg
import numpy as np
import math
import random
import time
import threading

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("command_proxy_example_server")
node.setupParameter("hz", 60)


#⬢⬢⬢⬢⬢➤ SIMULA UN TASK ASINCRONO CHE IMPIEGA RANDOM SECONDI PER FINIRE
def sleepAndResolve(message):
    if random.randint(0, 100) < 10:
        time.sleep(random.uniform(0.01, 5.0))
    print("Resolved...")
    message.addResponseData("pino", 1)
    message.addResponseData("aaa", 2)
    command_proxy_server.resolveCommand(message)
    return


def example_callback(message):
    print("Received", str(message))
    p = random.randint(0, 100)
    if p > 10:
        print("Resolving...")
        t = threading.Thread(target=sleepAndResolve, args=(message,))
        t.start()
    else:
        print("Rejecting...")
        command_proxy_server.rejectCommand(
            message, reject_message="RANDOM_ERROR!")


command_proxy_server = CommandProxyServer("pino")
command_proxy_server.registerCallback(example_callback)


while node.isActive():

    node.tick()
