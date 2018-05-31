#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32, Pose
from std_msgs.msg import String
from std_msgs.msg import Header
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf
import superros.transformations as transformations
from superros.logger import Logger

from superros.comm import RosNode
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.param.global_parameters import Parameters
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage
from rocup.taskmanager.task_manager_state_machine import TaskManagerSM
from rocup.proxy.proxy_message import SimpleMessage
import rocup.sfm.machines as machines
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import rospkg
from superros.comm import RosNode
import numpy as np
import math
import rosnode
import sys
import json
import random


if __name__ == '__main__':
    node = RosNode("task_manager")
    node.setupParameter("hz", 50)
    node.setHz(node.getParameter("hz"))

    task_name = "task_manager_test"

    robot_list = []
    sensor_list = []
    subtask_list = []

    instruction_list = [

        "system set repete:::False",
        "system sleep 2"

    ]

    ###############################################################################################
    ###############################################################################################
    ###############################################################################################

    tskm = TaskManagerSM(task_name)

    tskm.start(robot_list, sensor_list, instruction_list, subtask_list)
    try:
        while node.isActive():
            tskm.stepForward()
            node.tick()
    except rospy.ROSInterruptException:
        pass
