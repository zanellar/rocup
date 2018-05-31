#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
import math
import json
import numpy as np
import PyKDL
import superros.transformations as transformations
from superros.logger import Logger
from superros.comm import RosNode

from rocup.param.global_parameters import Parameters
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer
from rocup.partdb.cad import Scene, Shape, Face, STLUtils, Transform, Part, Terminal, PartCollection, Gearbox, Channel
from rocup.partdb.data_dictionaries import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse

from rocup.partdb.data_dictionaries import GearboxPartsData
from rocup.param.global_parameters import Parameters

#⬢⬢⬢⬢⬢➤ ROBOT
robot_name = Parameters.get("GRASSHOPPER_NAME")  # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
Hb_open = "\033[94m\033[1m\033[4m"
Hg_open = "\033[92m\033[1m\033[4m"
Hr_open = "\033[91m\033[1m\033[4m"
H_close = "\033[0m"
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class ConnectionsScanProxyServer(object):
    def __init__(self):
        self.command_proxy_server_name = "grasshopper_grasp_supervisor"
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
                        data = msg.getData("data")
                    except:
                        data = None
                    Trigger(data)
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


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


def Trigger(data=None):
    global is_task_active
    is_task_active = True
    is_done = False

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


if __name__ == '__main__':

    #⬢⬢⬢⬢⬢➤ NODE
    node = RosNode("grasshopper_grasp")

    node.setupParameter("hz", 10)  #
    node.setHz(node.getParameter("hz"))

    proxy_server = ConnectionsScanProxyServer()

    is_task_active = False
    is_done = False

    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ LOOP ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    #▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    while node.isActive():
        if is_task_active:
            if is_done:
                proxy_server.sendResponse(True)
                is_task_active = False
            else
                is_done = True
        node.tick()
