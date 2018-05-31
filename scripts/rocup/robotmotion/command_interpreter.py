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
from rocup.robots.controllers import RobotController, GeneralIKService, Robot
from rocup.robots.market import RobotMarket
from rocup.srv import IKService, IKServiceResponse
import superros.transformations as transformations
from superros.logger import Logger

from rocup.msg import *

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import tf
from scipy.interpolate import interp1d

from std_msgs.msg import String
from transitions import Machine

from rocup.param.global_parameters import Parameters


WORLD_FRAME_ID = Parameters.get("WORLD_FRAME_ID")


class CommandInterpreter(object):

    def __init__(self, robot_name="",  robot_object=None, callback=None):
        # tf Listener
        self.listener = tf.TransformListener()

        # robot
        self.robot_name = robot_name
        self.robot = robot_object

        # trajectory
        self.target = None
        self.trajectory_specs = ""

    def interpretCommand(self, command):

        try:
            if command.startswith("shape_"):
                shape = command.split("_")[1]
                self.target = self.robot.getShapeQ(shape)
                self.trajectory_specs = "linear_joints"
                self.command_valid = True

            # Generate a trajectory from joints coordinates
            # The trajectory is linear in the joints space
            #   Usage. "joints *,*,*,*,*,*"
            elif command.startswith("joints "):
                command_q = command.split(" ")[1].split(",")
                self.target = [float(q) for q in command_q]
                self.trajectory_specs = "linear_joints"
                self.command_valid = True

            # Generate a trajectory from current pose to a tf ID.
            # The trajectory is linear in the operational space
            #   Usage: "gototf <tf_name>"
            elif command.startswith("gototf "):
                tf = command.split(' ')[1]
                target = transformations.retrieveTransform(self.listener, WORLD_FRAME_ID, tf)
                self.target = target
                robot_base = self.robot.getBaseFrame()
                self.target = robot_base.Inverse() * self.target
                self.trajectory_specs = "linear_frames_to_free_joints"
                self.command_valid = True

            # Generate a trajectory from current pose to a tf ID
            # The trajectory is linear in the joints space BUT "free" in operational space
            #   Usage: "movetotf <tf_name>"
            elif command.startswith("movetotf "):
                tf = command.split(' ')[1]
                target = transformations.retrieveTransform(self.listener, WORLD_FRAME_ID, tf)
                self.target = target
                robot_base = self.robot.getBaseFrame()
                self.target = robot_base.Inverse() * self.target
                self.trajectory_specs = "free_frames_to_linear_joints"
                self.command_valid = True

            elif command.startswith("jumptotf "):
                tf = command.split(' ')[1]
                target = transformations.retrieveTransform(self.listener, WORLD_FRAME_ID, tf)
                self.target = target
                robot_base = self.robot.getBaseFrame()
                self.target = robot_base.Inverse() * self.target
                self.trajectory_specs = "target_frame_to_target_joints"
                self.command_valid = True
            # command string recognition feedback
            else:
                self.command_valid = False
        except:
            self.command_valid = False

        return self.target

    def isCommandValid(self):
        if self.target == None:
            return False
        return self.command_valid
