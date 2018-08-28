#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
import numpy as np
import time
import tf
import json
import math
import time
import PyKDL
import random
from PyKDL import Frame, Vector, Rotation

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Float64, Float32, Float64MultiArray

import superros.transformations as transformations
from superros.logger import Logger
from superros.comm import RosNode
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer


class SensorManager(object):

    def __init__(self, sensor_name):
        self.sensor_name = sensor_name
        self.message_proxy = SimpleMessageProxy()
        self.message_proxy.register(self.command_callback)
        self.last_msg = None

        # robot outer command
        self.command_server = CommandProxyServer(
            "{}_supervisor".format(self.sensor_name))
        self.command_server.registerCallback(
            self.command_action_callback)
        self.command_server_last_message = None

        # # robot inner message
        # self.inner_message_proxy = SimpleMessageProxy(
        #     "{}_inner_message".format(self.robot_name))

    # def _sendFeedback(self, msg):
    #     msg.setReceiver("{}_direct_commander".format(self.robot_name))
    #     msg.setData("type", "force")
    #     self.inner_message_proxy.send(msg)

    def update(self, msg):
        self.last_msg = msg

    def uploadResetPublisher(self, pub):
        self.reset_publisher = pub

    def command_action_callback(self, cmd):
        self.command_server_last_message = cmd
        cmd_msg = cmd.getSentMessage()
        self.command_callback(cmd_msg)

    def command_callback(self, msg):
        try:
            if msg.isValid():
                if msg.getReceiver() == "{}_supervisor".format(self.sensor_name):
                    command = msg.getCommand()
                    self.result_flg = True
                    if command == "reset":
                        self.reset_publisher.publish("")
                        time.sleep(0.5)
                        Logger.warning("Sensor '{}': Reset".format(self.sensor_name))
                    elif command == "filteron":
                        Logger.warning("Sensor '{}': Filter ON".format(self.sensor_name))
                        # TODO
                    elif command == "filteroff":
                        Logger.warning("Sensor '{}': Filter OFF".format(self.sensor_name))
                        # TODO
                    elif command == "filterreset":
                        Logger.warning("Sensor '{}': Filter Reset".format(self.sensor_name))
                        # TODO
                    else:
                        self.result_flg = False
                        Logger.error("INVALID input")

                    self._send_command_result(self.result_flg)

        except Exception as e:
            print(e)

    def _send_command_result(self, success):
        if self.command_server_last_message:
            if success:
                self.command_server.resolveCommand(
                    self.command_server_last_message)
            else:
                self.command_server.rejectCommand(
                    self.command_server_last_message)
            self.command_server_last_message = None
