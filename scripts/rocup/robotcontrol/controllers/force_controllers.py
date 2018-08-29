#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy

from rocup.msg import RobotFollow
from superros.logger import Logger
import superros.transformations as transformations
from superros.transformations import FrameVectorFromKDL, FrameVectorToKDL, ListToKDLVector, KDLVectorToList
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


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    SpringForceController    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class SpringForceController(object):

    DEFAULT_PARAMETERS = {
        "translation_mag": 0.0001,
        "rotation_mag": 0.003,
        "thresholds": [0.5, 0.5, 1]
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.is_active = False
        self.done = False

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict

        self.translation_mag = self._param("translation_mag", params_dict)
        self.rotation_mag = self._param("rotation_mag", params_dict)
        self.thresholds = self._param("thresholds", params_dict)

        # Force sensor
        forceSensorFrame = PyKDL.Frame()
        forceSensorFrame.M.DoRotX(np.pi)
        self.forceSensor = ForceSensor(relative_frame=forceSensorFrame,
                                       translation_mag=self.translation_mag,
                                       rotation_mag=self.rotation_mag)
        self.forceSensor.setTransThreshold(self.thresholds)
        self.last_force_msg = Twist()
        self.force_limit = [2, 2, 2]

    def _param(self, name, param_dict):
        if name in param_dict.keys():
            par = param_dict[name]
            self.params[name] = par
            return par
        else:
            return self.params[name]

    def _estimateDisturbance(self, sample):
        dist = np.array([0.0, 0.0, 0.0])
        fx = self.last_force_msg.linear.x
        fy = self.last_force_msg.linear.y
        fz = self.last_force_msg.linear.z
        forces = np.array([fx, fy, fz])
        for i in range(0, sample):
            dist += forces
        return (dist / sample)

    def setParameters(self, params_dict):
        try:
            self.translation_mag = self._param("translation_mag", params_dict)
            self.rotation_mag = self._param("rotation_mag", params_dict)
            self.thresholds = self._param("thresholds", params_dict)

            self.forceSensor.setMagnitude(self.translation_mag,
                                          self.rotation_mag)
            self.forceSensor.setTransThreshold(self.thresholds)
        except Exception as e:
            Logger.error(e)

    def getParameters(self):
        return self.params

    def reset(self):
        self.is_active = False
        self.done = False
        thr = self._estimateDisturbance(20) * 1.10
        self.setParameters({"thresholds": thr})
        Logger.warning("thresholds={}".format(thr))

    def start(self, data):
        self.is_active = True
        self.done = False

    def output(self, data):
        if "atift" in data.keys():
            self.last_force_msg = data["atift"]
            self.forceSensor.update(self.last_force_msg)

        current_tf = data["current_tf"]
        target_tf = data["target_tf"]

        if self.is_active:

            if self.done:
                print "\n\n\n CONTROLLER STOP \n\n\n"
                self.is_active = False
                return target_tf

            Tr = self.forceSensor.output()
            target_tf = current_tf * Tr

        return target_tf


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇    DampedForwardForceController    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class DampedForwardForceController(object):

    STANDARD_INPUT = {
        "axis": PyKDL.Vector()
    }

    DEFAULT_PARAMETERS = {
        "velocity": 0.0005,
        "damp_force_threshold": 1,
        "damp_magnitude": 0.995
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.is_active = False
        self.done = False

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict
        self.damp_force_threshold = self._param("damp_force_threshold",
                                                params_dict)
        self.damp_magnitude = self._param("damp_magnitude",
                                          params_dict)
        self.initial_velocity = self._param("velocity",
                                            params_dict)
        self.velocity = 0
        self.axis = PyKDL.Vector()
        self.last_force_msg = Twist()
        self.force_limit = [2, 2, 2]

    def _param(self, name, param_dict):
        if name in param_dict.keys():
            par = param_dict[name]
            self.params[name] = par
            return par
        else:
            return self.params[name]

    def _updateVelocity(self, force_data):
        axis_array = np.array(KDLVectorToList(self.axis))
        self.lin_force_array = np.array([force_data.linear.x,
                                         force_data.linear.y,
                                         force_data.linear.z])
        axial_force = np.linalg.norm(self.lin_force_array * axis_array)
        damp_coefficient = self.damp_magnitude
        # damp_coefficient = self.damp_magnitude / (axial_force / self.damp_force_threshold)
        if axial_force > self.damp_force_threshold:
            self.velocity = self.velocity * damp_coefficient

    def _estimateDisturbance(self, sample):
        dist = np.array([0.0, 0.0, 0.0])
        for i in range(0, sample):
            dist += self.lin_force_array * np.array(KDLVectorToList(self.axis))
            print("{}{}{}".format(self.lin_force_array,
                                  self.axis, dist))
        return np.linalg.norm((dist / sample))

    def setParameters(self, params_dict):
        try:
            self.damp_force_threshold = self._param("damp_force_threshold",
                                                    params_dict)
            self.damp_magnitude = self._param("damp_magnitude",
                                              params_dict)
            self.initial_velocity = self._param("velocity",
                                                params_dict)
            print(self.initial_velocity)

        except Exception as e:
            Logger.error(e)

    def getParameters(self):
        return self.params

    def reset(self):
        self.is_active = False
        self.done = False
        # self.velocity = 0
        # self.axis = PyKDL.Vector()
        thr = self._estimateDisturbance(20) * 1.3
        self.setParameters({"damp_force_threshold": thr})
        Logger.warning("damp_force_threshold={}".format(thr))

    def start(self, data):
        self.is_active = True
        self.done = False
        self.axis = ListToKDLVector(data["axis"])
        self.velocity = self.initial_velocity

    def output(self, data):
        if "atift" in data.keys():
            self.last_force_msg = data["atift"]
            self._updateVelocity(self.last_force_msg)

        target_tf = data["target_tf"]
        current_tf = data["current_tf"]

        if self.is_active:

            if self.done:
                print "\n\n\n CONTROLLER STOP \n\n\n"
                self.is_active = False
                return self.target_tf

            Tr = PyKDL.Frame(self.axis * self.velocity)
            print(self.velocity)
            target_tf = current_tf * Tr

        return target_tf


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    SpringTouchController    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class SpringTouchController(object):

    STANDARD_INPUT = {
        "axis": PyKDL.Vector()
    }

    DEFAULT_PARAMETERS = {
        "dampedforward_parameters": DampedForwardForceController.DEFAULT_PARAMETERS,
        "spring_parameters": SpringForceController.DEFAULT_PARAMETERS
    }

    def __init__(self, robot_name="", params_dict={}):
        self.robot_name = robot_name
        self.is_active = False
        self.done = False

        # Controller Parameters
        if params_dict == {} or params_dict == None:
            params_dict = self.DEFAULT_PARAMETERS

        self.params = params_dict

        self.spring_controller = SpringForceController()
        self.dampedforward_controller = DampedForwardForceController()
        self.last_force_msg = Twist()

    def _param(self, name, param_dict):
        if name in param_dict.keys():
            par = param_dict[name]
            self.params[name] = par
            return par
        else:
            return self.params[name]

    def setParameters(self, params_dict):
        try:
            self.dampedforward_controller.setParameters(self._param("dampedforward_parameters",
                                                                    params_dict))
            self.spring_controller.setParameters(self._param("spring_parameters",
                                                             params_dict))
        except Exception as e:
            Logger.error(e)

    def getParameters(self):
        return self.params

    def reset(self):
        self.is_active = False
        self.done = False
        self.dampedforward_controller.reset()
        self.spring_controller.reset()

    def start(self, data):
        self.is_active = True
        self.done = False
        self.dampedforward_controller.start(data)

    def output(self, data):
        if "atift" in data.keys():
            self.last_force_msg = data["atift"]

        if "wrist_ft_sensor" in data.keys():
            simple_message = data["wrist_ft_sensor"]
            self.last_force_msg = Twist()
            self.last_force_msg.linear.x = simple_message.getData("fx")
            self.last_force_msg.linear.y = simple_message.getData("fy")
            self.last_force_msg.linear.z = simple_message.getData("fz")
            self.last_force_msg.angular.x = simple_message.getData("tx")
            self.last_force_msg.angular.y = simple_message.getData("ty")
            self.last_force_msg.angular.z = simple_message.getData("tz")

        current_tf = data["current_tf"]
        target_tf = data["target_tf"]

        if self.is_active:

            if self.done:
                print "\n\n\n CONTROLLER STOP \n\n\n"
                self.is_active = False
                return self.target_tf

            Tr1 = self.dampedforward_controller.output(data)
            Tr2 = self.spring_controller.output(data)
            Tr = Tr2 * Tr1

            target_tf = current_tf * Tr

        return target_tf
