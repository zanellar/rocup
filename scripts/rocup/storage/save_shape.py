#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy

from std_msgs.msg import *


import random
import numpy
from numpy import *
import numpy as np
import math
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)
from superros.logger import Logger
import superros.transformations as transformations
from rocup.storage.mongo import MessageStorage
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import PyKDL
from PyKDL import Frame, Vector, Rotation
import threading

from std_msgs.msg import String


import tf
import cv2
import time


from rospkg import RosPack


class SaveShape():

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.joints_state_sub = rospy.Subscriber("/{}/joint_states".format(self.robot_name),
                                                 JointState, self._jointStateCallback)
        self.message_database = MessageStorage()
        self.joint_states_msg = JointState()

    def _jointStateCallback(self, msg):
        self.joint_states_msg = msg

    def save(self, input_name):
        try:
            self.message_database.replace(input_name, self.joint_states_msg)
        except:
            pass

    def delete(self, input_name):
        try:
            self.message_database.deleteByName(
                input_name, self.joint_states_msg)
        except:
            pass

    def saveFrame(self, frame, input_name):
        try:
            pose = transformations.KDLToPose(frame)
            self.message_database.replace(input_name, pose)
        except:
            pass
