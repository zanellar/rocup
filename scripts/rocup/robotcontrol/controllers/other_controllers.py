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
        # @@@ add here the parameters...
        "gain": 1.0
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.is_active = False
        self.done = False
        self.params = {}

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict

        self.gain = self._param("gain",
                                params_dict)
        # @@@ add here the other parameters...

    def _param(self, name, param_dict):
        if name in param_dict.keys():
            par = param_dict[name]
            self.params[name] = par
            return par
        else:
            return self.params[name]

    def setParameters(self, params_dict=None, standard_index="default"):
        ''' Set the parameters stored by name in the dictionary "params_dict".'''
        if params_dict is not None:
            try:
                self.gain = self._param("gain",
                                        params_dict)
                # @@@ add here the other parameters...
            except Exception as e:
                Logger.error(e)

    def getParameters(self):
        ''' Returns the parameters stored by name in a dictionary.'''
        return self.params

    def reset(self):
        ''' Reset the variables and deactivate the control'''
        self.is_active = False
        self.done = False
        # @@@ code here ...
        pass

    def start(self, data):
        ''' Activate the control action. Require a data dictionary with 
        inputs values (it can be empty if no inputs values is needed 
        for this controller).'''
        self.is_active = True
        self.done = False
        # @@@ code here ...
        pass

    def output(self, data):
        ''' Return the target frame. Require a data dictionary with 
        inputs values with the current reference frame "target_tf" that 
        will be transformed by the control action.'''
        Tr = PyKDL.Frame()
        current_tf = data["current_tf"]
        target_tf = data["target_tf"]
        # @@@ code here ...

        if self.is_active:

            if self.done:
                print "\n\n\n CONTROLLER STOP \n\n\n"
                self.is_active = False
                return self.target_tf

            Tr = PyKDL.Frame()  # identyty
            target_tf = target_tf * Tr

            # @@@ code here ...
        # @@@ code here ...

        return target_tf
