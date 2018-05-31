#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This example demonstrates a very basic use of flowcharts: filter data,
displaying both the input and output of the filter. The behavior of
he filter can be reprogrammed by the user.
Basic steps are:
  - create a flowchart and two plots
  - input noisy data to the flowchart
  - flowchart connects data to the first plot, where it is displayed
  - add a gaussian filter to lowpass the data, then display it in the second plot.
"""


import numpy as np
import rospy

from rocup.msg import RobotFollow
from superros.logger import Logger
import superros.transformations as transformations
from rocup.utils.devices import ForceSensor, Joystick
from rocup.param.global_parameters import Parameters
from rocup.robots.controllers import RobotStatus
from rocup.proxy.target_follower_proxy import TargetFollowerProxy
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
import PyKDL
import copy
import math
import tf


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    NeutralController   ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class NeutralController(object):

    STANDARD_INPUT = {
    }

    DEFAULT_PARAMETERS = {
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.params = {}

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict

    def _param(self, name, param_dict):
        if name in param_dict.keys():
            par = param_dict[name]
            self.params[name] = par
            return par
        else:
            return self.params[name]

    def setParameters(self, params_dict=None, standard_index="default"):
        pass

    def getParameters(self):
        return self.params

    def reset(self):
        pass

    def start(self, data):
        pass

    def update(self, msg, feedback_source=""):
        pass

    def output(self, data):
        Tr = PyKDL.Frame()
        target_tf = data["target_tf"]
        return target_tf
