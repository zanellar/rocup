#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
import numpy as np
import time
import tf
import math
import time
import PyKDL
import random
from PyKDL import Frame, Vector, Rotation

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Float64, Float32, Float64MultiArray

import superros.transformations as transformations
from superros.logger import Logger
from superros.comm import RosNode
from rocup.sensors.sensor_manager import SensorManager

import json


def callback(msg):
    global sensor
    msg_str = msg
    sensor.update(msg_str)


if __name__ == '__main__':
    node = RosNode("compass_manager_node")
    node.setupParameter("hz", 250)
    node.setHz(node.getParameter("hz"))

    sensor_name = "compass"
    sensor = SensorManager(sensor_name)
    node.createSubscriber("/compass/pose", String, callback)

    try:
        while node.isActive():
            node.tick()
    except rospy.ROSInterruptException:
        pass
