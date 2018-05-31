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
import json
from scipy.interpolate import interp1d

from std_msgs.msg import String
from transitions import Machine

from rocup.param.global_parameters import Parameters


class SimpleMessage(object):
    _SEP_ = "_SEP_"
    _PAR_ = "_PAR_"
    _VAL_ = "_VAL_"

    def __init__(self, command="", receiver="", sender="", json_string=None):
        if json_string:
            self.data = json.loads(json_string)
        else:
            self.data = {}
            self.data["receiver"] = receiver
            self.data["command"] = command
            self.data["sender"] = sender
            self.data["_is_response"] = False

    def isValid(self):
        if "receiver" in self.data and "command" in self.data:
            return True
        return False

    def setData(self, key, val):
        self.data[key] = val

    def getData(self, key, tp=None):
        if key in self.data:
            data = self.data[key]
            try:
                data = data.strip()
            except:
                pass

            if tp == None:
                return data
            else:
                return tp(data)
        return None

    def setReceiver(self, receiver):
        self.setData("receiver", receiver)

    def getReceiver(self):
        return self.getData("receiver")

    def setCommand(self, command):
        self.setData("command", command)

    def getCommand(self):
        return self.getData("command")

    def setSender(self, sender):
        self.setData("sender", sender)

    def getSender(self):
        return self.getData("sender")

    @staticmethod
    def messageFromString(msg_str):
        return SimpleMessage(json_string=msg_str)

    @staticmethod
    def messageFromDict(d):
        return SimpleMessage.messageFromString(json.dumps(d))

    def toString(self):
        return json.dumps(self.data)


class SimpleMessageProxy(object):
    MESSAGE_PROXY_NAMESPACE = "/simple_message_stream"

    def __init__(self,  name='common'):

        self.name = name
        self.stream = SimpleMessageProxy.MESSAGE_PROXY_NAMESPACE + "/" + name
        self.receiver_ext_callback = []

        self.out_stream_pub = rospy.Publisher(self.stream,
                                              String,
                                              queue_size=1)
        self.in_stream_sub = rospy.Subscriber(self.stream,
                                              String,
                                              self._message_callback,
                                              queue_size=1000)

    def _message_callback(self, msg):
        msg_str = msg.data
        message = SimpleMessage.messageFromString(msg.data)

        for c in self.receiver_ext_callback:
            c(message)

    def send(self, message):
        msg = String()
        msg.data = message.toString()
        self.out_stream_pub.publish(msg)

    def register(self, ext_callback):
        self.receiver_ext_callback.append(ext_callback)
