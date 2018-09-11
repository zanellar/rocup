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
import json


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


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇   PIDCompassController   ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class PIDCompassController(object):

    STANDARD_INPUT = {
    }

    DEFAULT_PARAMETERS = {
        # @@@ add here the parameters...
        "kdp": 0.0,
        "krp": 0.0,
        "sat_pos_error": 1.0,
        "sat_rot_error": 1.0
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.is_active = False
        self.done = False
        self.last_compass_msg = None
        self.target_obj = None
        self.params = {}

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict

        self.kdp = self._param("kdp",
                               params_dict)
        self.sat_rot_error = self._param("sat_rot_error",
                                         params_dict)

        self.sat_pos_error = self._param("sat_pos_error",
                                         params_dict)
        self.krp = self._param("krp",
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
                self.kdp = self._param("kdp",
                                       params_dict)
                self.sat_rot_error = self._param("sat_rot_error",
                                                 params_dict)

                self.sat_pos_error = self._param("sat_pos_error",
                                                 params_dict)
                self.krp = self._param("krp",
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

        # self.target_obj = data["object"]

        # @@@ code here ...
        pass

    @staticmethod
    def satFunc(x, xsat):
        y = xsat * (1.0 / (1.0 + math.e**(-x)) - 0.5)
        return y

    def output(self, data):
        ''' Return the target frame. Require a data dictionary with 
        inputs values with the current reference frame "target_tf" that 
        will be transformed by the control action.'''

        current_tf = data["current_tf"]
        target_tf = data["target_tf"]

        if "compass" in data.keys():
            self.last_compass_msg = json.loads(data["compass"].data)

        if self.last_compass_msg is None:
            return target_tf

        # obj_message = self.last_compass_msg["object"]

        # if obj_message != self.target_obj:
        #     return target_tf

        # @@@ code here ...

        if self.is_active:
            if self.last_compass_msg is None:
                return target_tf

            err_d = self.last_compass_msg["position"]
            angle = self.last_compass_msg["angle"]

            if self.done:
                print("\n\n\n CONTROLLER STOP \n\n\n")
                self.is_active = False
                return target_tf

            Tr = PyKDL.Frame()
            x_ctrl = self.kdp * err_d[0]
            y_ctrl = self.kdp * err_d[1]
            x_ctrl = self.satFunc(x_ctrl, self.sat_pos_error)
            y_ctrl = self.satFunc(y_ctrl, self.sat_pos_error)
            Tr.p = PyKDL.Vector(x_ctrl, y_ctrl, 0.0)

            direction_flg = False

            if direction_flg:
                v0 = PyKDL.Vector(1.0, 0.0, 0.0)
            else:
                v0 = PyKDL.Vector(0.0, 1.0, 0.0)
            vx = math.cos(angle)
            vy = math.sin(angle)
            v = PyKDL.Vector(vx, vy, 0.0)
            v_prj = PyKDL.dot(v, v0)

            if direction_flg:
                sgn_rot = 1.0 if angle < math.pi else -1.0
                err_v = sgn_rot * abs(v_prj - 1.0)
            else:
                positive_rotation = (angle < math.pi / 2 and angle > 0) or (angle < math.pi * 3 / 2 and angle >= math.pi)
                sgn_rot = 1.0 if positive_rotation else -1.0
                err_v = sgn_rot * abs(v_prj)

            theta_ctrl = self.krp * err_v
            theta_ctrl = self.satFunc(theta_ctrl, self.sat_rot_error)
            Tr.M = PyKDL.Rotation.RotZ(theta_ctrl)

            print(x_ctrl, y_ctrl, theta_ctrl)
            target_tf = target_tf * Tr

            # @@@ code here ...
        # @@@ code here ...

        return target_tf
