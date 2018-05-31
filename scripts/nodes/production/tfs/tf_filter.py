#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf
import superros.transformations as transformations
from superros.logger import Logger

from superros.comm import RosNode
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer

import message_filters
import rospkg
import numpy as np
import math
import sys
import random


class TfFilter(object):
    def __init__(self):
        self.filtered_tf_map = {}
        self.filtered_tf_slots = {}
        self.current_tf_name = ""

        self.default_sls = 100

        self.command_server = CommandProxyServer("tf_filter_supervisor")
        self.command_server.registerCallback(self.command_action_callback)
        self.command_server_last_message = None

        self.message_proxy = SimpleMessageProxy()
        self.message_proxy.register(self.command_callback)

    def resetFiltering(self):
        Logger.warning("FILTER RESET")
        self.filtered_tf_map = {}
        self.filtered_tf_slots = {}
        # self._send_command_result(True)

    def clearFiltering(self, name):
        Logger.warning("FILTER CLEAR: {}".format(name))
        self.filtered_tf_map[name] = []
        self.filtered_tf_slots[name] = 0
        # self._send_command_result(True)

    def startFiltering(self, name, slot_size):
        Logger.warning("FILTER START: {}".format(name))
        self.current_tf_name = name
        self.filtered_tf_slots[name] = slot_size
        if name not in self.filtered_tf_map:
            self.filtered_tf_map[name] = []

    def stopFiltering(self):
        Logger.warning("FILTER STOP")
        self.current_tf_name = ''
        # self._send_command_result(True)

    def updateAndBroadcast(self, node, current_tf, size):
        if self.current_tf_name != '':
            if current_tf:
                if self.filtered_tf_slots[self.current_tf_name] > 0:
                    self.filtered_tf_slots[self.current_tf_name] -= 1
                    print(self.current_tf_name, self.filtered_tf_slots[self.current_tf_name])

                    if len(self.filtered_tf_map[self.current_tf_name]) < size:
                        self.filtered_tf_map[self.current_tf_name].append(current_tf)
                    else:
                        Logger.warning("TF Buffer for '{}' is full! Clear it...".format(current_tf_name))
                else:
                    self._send_command_result(True)

        p = np.array([0, 0, 0])
        q = np.array([0, 0, 0, 0])
        for tf_name, tf_list in self.filtered_tf_map.iteritems():
            if len(tf_list) <= 0:
                continue
            for tf in tf_list:
                newp = np.array([tf.p.x(), tf.p.y(), tf.p.z()])
                qx, qy, qz, qw = tf.M.GetQuaternion()
                newq = np.array([qx, qy, qz, qw])
                p = p + newp
                q = q + newq
            p = p / float(len(tf_list))
            q = q / float(len(tf_list))
            q = q / np.linalg.norm(q)

            filtered_tf = PyKDL.Frame()
            filtered_tf.p = PyKDL.Vector(p[0], p[1], p[2])
            filtered_tf.M = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])
            node.broadcastTransform(filtered_tf,
                                    tf_name + "_filtered",
                                    node.getParameter("world_tf"),
                                    node.getCurrentTime())

        # return filtered_tf

    def _send_command_result(self, success):
        if self.command_server_last_message:
            print("\nTask Ended!!! \t Success: {}".format(success))
            if success:
                self.command_server.resolveCommand(
                    self.command_server_last_message)
            else:
                self.command_server.rejectCommand(
                    self.command_server_last_message)
            self.command_server_last_message = None

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ CALLBACKS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def command_action_callback(self, cmd):
        self.command_server_last_message = cmd
        cmd_msg = cmd.getSentMessage()
        self.command_callback(cmd_msg)

    def command_callback(self, msg):
        try:
            if msg.isValid():
                if msg.getReceiver() == "tf_filter_supervisor":
                    command = msg.getCommand()
                    Logger.log(command)
                    if command == 'start':
                        sls = msg.getData('slot_size', float)
                        if sls is None:
                            sls = self.default_sls
                        self.startFiltering(msg.getData('tf_name'), slot_size=sls)
                    elif command == 'stop':
                        self.stopFiltering()
                        self._send_command_result(True)
                    elif command == 'clear':
                        self.clearFiltering(msg.getData('tf_name'))
                        self._send_command_result(True)
                    elif command == 'clearstart':
                        self.clearFiltering(msg.getData('tf_name'))
                        sls = msg.getData('slot_size', float)
                        if sls is None:
                            sls = self.default_sls
                        self.startFiltering(msg.getData('tf_name'), slot_size=sls)
                    elif command == 'reset':
                        self.resetFiltering()
                        self._send_command_result(True)
                    else:
                        Logger.error("INVALID input")
                        self._send_command_result(False)

        except Exception as e:
            print(e)
            self._send_command_result(False)


#⬢⬢⬢⬢⬢➤ NODE
if __name__ == '__main__':

    node = RosNode("tf_filter")

    node.setupParameter("hz", 30)
    node.setupParameter("world_tf", "world")
    node.setupParameter("target_tf", "target")
    node.setupParameter("tf_buffer_size", 1300)
    node.setupParameter("tf_slot_size", 100)
    node.setHz(node.getParameter("hz"))

    tffilter = TfFilter()

    try:
        while node.isActive():
            if tffilter.current_tf_name != '':
                current_tf = None
                while not current_tf:
                    current_tf = node.retrieveTransform(tffilter.current_tf_name,
                                                        node.getParameter("world_tf"),
                                                        rospy.Time(0))

                size = node.getParameter("tf_buffer_size")
                tffilter.updateAndBroadcast(node, current_tf, size)

            node.tick()
    except rospy.ROSInterruptException:
        pass
