#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
import math
import json
import numpy as np
import time
import PyKDL
import superros.transformations as transformations
from superros.logger import Logger
from superros.comm import RosNode

from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from rocup.param.global_parameters import Parameters

#⬢⬢⬢⬢⬢➤ ROBOT
Hb_open = "\033[94m\033[1m\033[4m"
Hg_open = "\033[92m\033[1m\033[4m"
Hr_open = "\033[91m\033[1m\033[4m"
H_close = "\033[0m"

STANDARD_HEIGHT = -0.4  # [m]
ROBOT_NAME = "comau_smart_six"

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class ConnectionsScanProxyServer(object):
    def __init__(self):
        self.command_proxy_server_name = "gripper_supervisor"
        self.command_server = CommandProxyServer(self.command_proxy_server_name)
        self.command_server.registerCallback(self.command_action_callback)
        self.command_server_last_message = None

        self.message_proxy = SimpleMessageProxy()
        self.message_proxy.register(self.command_callback)

    def command_action_callback(self, cmd):
        self.command_server_last_message = cmd
        cmd_msg = cmd.getSentMessage()
        self.command_callback(cmd_msg)

    def command_callback(self, msg):
        try:
            if msg.isValid():
                if msg.getReceiver() == self.command_proxy_server_name:
                    command = msg.getCommand()
                    try:
                        dist = float(command)
                    except Exception as e:
                        Logger.error(e)
                        self.sendResponse(False)
                    Trigger(dist)
                    Logger.log(self.command_proxy_server_name.replace("_supervisor", "") + ": " + command)
                else:
                    self.command_server_last_message = None
            else:
                self.command_server_last_message = None
                Logger.error(self.command_proxy_server_name.replace("_supervisor", "") + ": invalid message!!")
                self.sendResponse(False)

        except Exception as e:
            self.sendResponse(False)

    def sendResponse(self, success):
        if self.command_server_last_message:
            print("\n" + self.command_proxy_server_name.replace("_supervisor", "")
                  + ": " + Hg_open + "Task Ended!!!" + H_close + " \t Success: {}".format(success))
            if success:
                self.command_server.resolveCommand(
                    self.command_server_last_message)
            else:
                self.command_server.rejectCommand(
                    self.command_server_last_message)
            self.command_server_last_message = None


def getFrame(frame_id, parent_id="world"):
    tf = None
    ct = 50
    while tf is None or ct > 50:
        ct -= 1
        try:
            tf = node.retrieveTransform(frame_id=frame_id,
                                        parent_frame_id=parent_id,
                                        time=-1)
        except Exception as e:
            tf = None
            print "Waiting for tf..."
    return tf

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


def _setGripper(p, v=50, e=50):
    global gripper_pub
    js = JointState()
    js.name = []
    js.position = [p]
    js.velocity = [v]
    js.effort = [e]
    gripper_pub.publish(js)


def Trigger(data=None):
    global is_task_active
    _setGripper(data)
    is_task_active = True
    is_done = False

# def gripper_state_cb(msg):
#     global is_done, aperture
#     aperture = abs(msg.position[0]) + abs(msg.position[1])
#     if True:  # aperture < target_aperture * 1.1 and aperture > target_aperture * 0.9:
#         is_done = True

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


if __name__ == '__main__':

    #⬢⬢⬢⬢⬢➤ NODE
    node = RosNode("subtask_grasp")

    node.setupParameter("hz", 10)  #
    node.setHz(node.getParameter("hz"))

    proxy_server = ConnectionsScanProxyServer()

    is_task_active = False
    is_done = False

    gripper_pub = node.createPublisher("/schunk_pg70/joint_setpoint", JointState)
    # node.createSubscriber("/schunk_pg70/joint_states", JointState, gripper_state_cb)

    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ LOOP ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    while node.isActive():
        target_tf = None
        if is_task_active:
            if is_done:
                proxy_server.sendResponse(True)
            else:
                is_done = True
        node.tick()
