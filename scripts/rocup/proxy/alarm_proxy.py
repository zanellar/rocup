#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy
from numpy import *
from numpy.linalg import *
import math
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)
import PyKDL
from PyKDL import Frame, Vector, Rotation
import superros.transformations as transformations
from superros.logger import Logger

from rocup.msg import *

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import tf
from scipy.interpolate import interp1d

from std_msgs.msg import String

from rocup.param.global_parameters import Parameters


class AlarmProxy(object):

    NONE_ALARM = 0
    UNKNOWN_ALARM = -100
    NODE_DIED = -101

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.alarm_ext_callback = None
        self.alarm_reset_ext_callback = None
        self.alarm_pub = rospy.Publisher('/{}/alarm'.format(self.robot_name),
                                         String, queue_size=1)
        self.alarm_sub = rospy.Subscriber("/{}/alarm".format(self.robot_name),
                                          String, self._alarm_callback)
        self.alarm_rest_pub = rospy.Publisher('/{}/alarm_reset'.format(self.robot_name),
                                              String, queue_size=1)
        self.alarm_sub = rospy.Subscriber("/{}/alarm_reset".format(self.robot_name),
                                          String, self._alarm_reset_callback)

    def _alarm_callback(self, msg):
        try:
            alarm_type = int(msg.data)
        except:
            alarm_type = AlarmProxy.UNKNOWN_ALARM

        if self.alarm_ext_callback:
            self.alarm_ext_callback(alarm_type)

    def _alarm_reset_callback(self, msg):
        if self.alarm_reset_ext_callback:
            self.alarm_reset_ext_callback()

    def resetAlarm(self):
        msg_str = String()
        self.alarm_rest_pub.publish(msg_str)

    def setAlarm(self, alarm_type=None):
        try:
            int(alarm_type)
            isAnInt = True
        except:
            isAnInt = False

        if not alarm_type or not isAnInt:
            alarm_type = AlarmProxy.UNKNOWN_ALARM
        msg_str = String()
        msg_str.data = str(alarm_type)
        self.alarm_pub.publish(msg_str)

    def registerResetCallback(self, callback):
        self.alarm_reset_ext_callback = callback

    def registerAlarmCallback(self, callback):
        self.alarm_ext_callback = callback
