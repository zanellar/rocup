#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)
import PyKDL

import tf
from scipy.interpolate import interp1d
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_msgs.msg import *

from transitions import Machine

from rocup.srv import IKService, IKServiceResponse
from superros.logger import Logger

import actionlib
from rocup.msg import *
from rocup.msg import RobotMotionResult, RobotMotionFeedback, RobotMotionAction, RobotMotionGoal
from robot_motion_state_machine import RobotMotionSM
from actionlib_msgs.msg import GoalStatus


def goalStatusString(state):
    for member in GoalStatus.__dict__.iteritems():
        if state == getattr(GoalStatus, member[0]):
            return member[0]


class RobotMotionClient:

    FEEDBACK = 1
    RESULT = 2

    def __init__(self, robot_name, ext_callback=None):
        action_name = robot_name + "_action"
        self.action_client = actionlib.SimpleActionClient(action_name,
                                                          RobotMotionAction)
        self.action_client.wait_for_server()
        self.ext_callback = ext_callback
        self.current_goal = None
        self.current_feedback = None
        self.progress = -1
        self.q = []
        self.current_result = None
        self.done_flg = True

    def generateNewGoal(self, goal):
        """ Genera un nuovo goal  e lo invia al server"""
        self.done_flg = False
        if goal:
            self.current_goal = RobotMotionGoal(robot_command=goal)
            self.action_client.send_goal(self.current_goal,
                                         done_cb=self.done_callback,
                                         active_cb=self.active_callback,
                                         feedback_cb=self.feedback_callback)
        else:
            Logger.error("Empty Goal")

    def getCurrentState(self):
        """ Ritorna la stringa che rappresenta lo stato corrente del Goal """
        return goalStatusString(self.action_client.get_state())

    def feedback_callback(self, feedback):
        """ Callback chiamata sui feedback """
        if self.ext_callback:
            self.ext_callback(msg=feedback, cb_type=RobotMotionClient.FEEDBACK)
        else:
            Logger.error("External callback for FEEDBACK update NOT found!")
        self.current_feedback = feedback
        self.progress = feedback.progress
        # Logger.log("feesdback  : \n \n {} \n\n".format(feedback))

    def done_callback(self, status, result):
        """ Callback chiamata al completamento del goal """
        if self.ext_callback:
            self.ext_callback(msg=result, cb_type=RobotMotionClient.RESULT)
        else:
            Logger.error("External callback for RESULT update NOT found!")
        self.done_flg = True
        self.current_result = result
        # Logger.log("Done !! : {} \n \n {} \n\n".format(
        #     goalStatusString(status), result))

    def active_callback(self):
        """ Callback chiamata appena il Goal diventa attivo sul server """
        # Logger.log("The goal is active!")


class RobotMotionServer:

    def __init__(self, robot_name):

        # action server set up
        action_name = robot_name + "_action"
        self.action_server = actionlib.SimpleActionServer(action_name,
                                                          RobotMotionAction,
                                                          auto_start=False)
        self.action_server.register_goal_callback(self.as_goal_callback)
        self.action_server.register_preempt_callback(self.as_preempt_callback)
        self.action_server.start()

        # state machine set up
        self.robot_sm = RobotMotionSM(robot_name=robot_name)
        self.robot_sm.registerFeedbackCallback(self.sm_feedback_callback)
        self.robot_sm.registerResultCallback(self.sm_result_callback)

    #
    # ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢  Action Server Callbacks ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢

    def as_preempt_callback(self):
        """ Callback chiamata quando c'e' gia un Goal attivo e ne arriva uno nuovo """
        if self.action_server.is_active():
            self.action_server.set_preempted()
            self._setNewGoal(self.action_server.accept_new_goal())
            Logger.log("Preemption! Last goal deleted!")

    def as_goal_callback(self):
        """ Callback chiamata SEMPRE all'arrivo di un nuovo Goal """
        if not self.action_server.is_active():
            self._setNewGoal(self.action_server.accept_new_goal())
        else:
            Logger.warning("New goal not accepted! Another goal is active")

    def _setNewGoal(self, goal):
        """ Set the new goal and update the model"""
        if goal:
            self.robot_sm.start(goal.robot_command)
        else:
            Logger.error("Empty Goal")

    #
    # ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ State Machine Callbacks ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢

    def sm_feedback_callback(self, feedback=None):
        self.action_server.publish_feedback(feedback)

    def sm_result_callback(self, result=None):
        if self.action_server.is_active():
            self.action_server.set_succeeded(result)

        # if result.success:
        #     Logger.log("Goal Executed!")
        # else:
        #     Logger.log("Goal -NOT- Executed")

    #
    # ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ SM Trigger ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢

    def stepForward(self):
        self.robot_sm.stepForward()
