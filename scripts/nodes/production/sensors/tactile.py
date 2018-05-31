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
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Float64, Float32, Float64MultiArray


import superros.transformations as transformations
from superros.logger import Logger
from superros.comm import RosNode
from rocup.sensors.sensor_manager import SensorManager


def tactile_callback(msg):
    global sensor
    sensor.update(msg)


if __name__ == '__main__':
    node = RosNode("tactile_supervisor_node")
    node.setupParameter("hz", 250)
    node.setHz(node.getParameter("hz"))

    sensor_name = "tactile"
    sensor = SensorManager(sensor_name)
    reset_publisher = node.createPublisher("/tactile_reset", String)
    sensor.uploadResetPublisher(reset_publisher)
    node.createSubscriber("/tactile", Float64MultiArray, tactile_callback)

    try:
        while node.isActive():
            node.tick()
    except rospy.ROSInterruptException:
        pass
