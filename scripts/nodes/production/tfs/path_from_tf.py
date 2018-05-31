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

import superros.transformations as transformations
from superros.logger import Logger

from superros.comm import RosNode

import message_filters
import rospkg
import numpy as np
import math


class Path(object):

    def __init__(self):
        self.nodes = []

    def append(self, pnode):
        self.nodes.append(pnode)

    def size(self):
        return len(self.nodes)

    def getFrame(self, index):
        path_node = self.nodes[index]
        if path_node.type == 'base' or index == 0:
            return path_node.frame
        else:
            return self.getFrame(index - 1) * path_node.frame


class PathNode(object):

    def __init__(self, frame, tp):
        self.frame = frame
        self.type = tp


#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("path_from_tf")

node.setupParameter("hz", 5)
node.setupParameter("path_name", "")
node.setupParameter("target", "comau_smart_six/base_link")
node.setupParameter("base_frame", "comau_smart_six/link3")
node.setupParameter(
    "path_file", "/home/daniele/Scrivania/new_wires_ws/src/rocup/data/paths/test_path.txt")


path_name = node.getParameter("path_name")
if path_name == "":
    path_name = node.getParameter("target")


#⬢⬢⬢⬢⬢➤ LOAD FILE
file = node.getParameter("path_file")
data = np.loadtxt(file,  usecols=[0, 1, 2, 3, 4, 5])
types = np.loadtxt(file, usecols=[6], dtype=str)
path = Path()
print(data.shape)
for row in range(0, data.shape[0]):
    path_node = PathNode(
        transformations.KDLFromArray(data[row, :]), types[row])
    path.append(path_node)


while node.isActive():

    base_frame = node.retrieveTransform(node.getParameter(
        "target"), node.getParameter("base_frame"), -1)

    for i in range(0, path.size()):
        frame = path.getFrame(i)
        node.broadcastTransform(
            frame,
            "{}_{}".format(path_name, i),
            node.getParameter("target"),
            node.getCurrentTime()
        )

    # marker_pub.publish(plane)
    node.tick()
