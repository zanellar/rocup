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
from rocup.msg import RobotFollow, RobotFollowStatus

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import tf
from scipy.interpolate import interp1d

from std_msgs.msg import String

from tf_conversions import posemath
from rocup.param.global_parameters import Parameters


class TargetFollowerProxy(object):

    SOURCE_DIRECT = 1
    SOURCE_TRAJECTORY = 2

    TYPE_POSE = 11
    TYPE_JOINTS = 12

    def __init__(self, robot_name):

        self.robot_name = robot_name

        # command
        self.follow_pub = rospy.Publisher('/{}/target_to_follow'.format(self.robot_name),
                                          RobotFollow, queue_size=1)
        self.follow_msg = RobotFollow()

        # feeedback
        self.alarm_status = False
        self.moving_status = False
        self.tool_status = None
        self.target_source_status = None
        self.alarm_ext_callback = None
        self.follow_sub = rospy.Subscriber('/{}/follow_status'.format(self.robot_name), RobotFollowStatus,
                                           self._status_callback, queue_size=1)

    def _status_callback(self, msg):
        self.alarm_status = msg.alarm
        self.moving_status = msg.moving
        self.tool_status = msg.tool
        self.target_source_status = msg.target_source
        if self.alarm_status == True:
            if self.alarm_ext_callback is not None:
                self.alarm_ext_callback(True)

    def _publishToFollow(self, follow_msg, sleep_time=0):
        self.follow_pub.publish(follow_msg)
        if sleep_time > 0:
            time.sleep(sleep_time)

    def setTarget(self, target, target_source, target_type):
        """ Set the robot 'target' as PyKDL frame or as joints list.
            Specify for such target the type (TYPE_POSE or TYPE_JOINTS)
            and the source (SOURCE_DIRECT or SOURCE_TRAJECTORY)"""
        # action
        self.follow_msg.action = RobotFollow.ACTION_SETTARGET
        # target type
        if target_type == self.TYPE_POSE:
            self.follow_msg.target_type = RobotFollow.TARGET_IN_POSE
            self.follow_msg.target_pose = posemath.toMsg(target)
        elif target_type == self.TYPE_JOINTS:
            self.follow_msg.target_type = RobotFollow.TARGET_IN_JOINT
            self.follow_msg.target_joint = target
        # target source
        if target_source == self.SOURCE_DIRECT:
            self.follow_msg.target_source = RobotFollow.TARGET_FROM_EXTERNAL_FEEDBACK
        elif target_source == self.SOURCE_TRAJECTORY:
            self.follow_msg.target_source = RobotFollow.TARGET_FROM_TRAJECTORY_PLANNER
        # publish
        self._publishToFollow(self.follow_msg)

    def setTool(self, tool):
        """ Set the robot 'tool' as PyKDL frame"""
        self.follow_msg.action = RobotFollow.ACTION_SETTOOL
        self.follow_msg.tool = posemath.toMsg(tool)
        self._publishToFollow(self.follow_msg)

    def setSource(self, source):
        """ Set the target 'source' as string """
        self.follow_msg.action = RobotFollow.ACTION_SETSOURCE
        if source == self.SOURCE_TRAJECTORY:
            self.follow_msg.target_source = RobotFollow.TARGET_FROM_TRAJECTORY_PLANNER
        elif source == self.SOURCE_DIRECT:
            self.follow_msg.target_source = RobotFollow.TARGET_FROM_EXTERNAL_FEEDBACK
        else:
            Logger.error("Invalid Source")
            return
        self._publishToFollow(self.follow_msg, sleep_time=0.1)

    def setAlarm(self):
        """ Set the Allarm"""
        self.follow_msg.action = RobotFollow.ACTION_ALARM
        self._publishToFollow(self.follow_msg)

    def resetAlarm(self):
        """ Reset the Allarm"""
        self.follow_msg.action = RobotFollow.ACTION_ALARMRESET
        self._publishToFollow(self.follow_msg)

    def registerAlarmCallback(self, callback):
        """ Register an external callback for robot alarm """
        self.alarm_ext_callback = callback
