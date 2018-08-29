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
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy


def ft_callback(msg):
    global message_proxy
    msg_str = msg
    message = SimpleMessage(sender="wrist_ft_sensor")
    message.setData("fx", msg.linear.x)
    message.setData("fy", msg.linear.y)
    message.setData("fz", msg.linear.z)
    message.setData("tx", msg.angular.x)
    message.setData("ty", msg.angular.y)
    message.setData("tz", msg.angular.z)
    message_proxy.send(message)


if __name__ == '__main__':
    node = RosNode("atift_manager_node")
    node.setupParameter("hz", 250)
    node.setHz(node.getParameter("hz"))

    node.createSubscriber("/atift", Twist, ft_callback)
    message_proxy = SimpleMessageProxy(name="feedbacks_stream")

    try:
        while node.isActive():
            node.tick()
    except rospy.ROSInterruptException:
        pass
