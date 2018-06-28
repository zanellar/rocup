#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import math
import time
import sys
import json
import random
from superros.comm import RosNode
import superros.transformations as transformations
from superros.logger import Logger

from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.param.global_parameters import Parameters
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage
from rocup.proxy.proxy_message import SimpleMessage
import rocup.sfm.machines as machines
import numpy as np


class SFMachineTaskManagerDefinition(object):

    def __init__(self, task_name):
        self.task_name = task_name

        self.active_command = None
        self.instruction_index = 0
        self.task_end = False
        self.last_action_success = False
        self.repete = True
        self.keep_tray_trajectory = False

        self.robot_name_list = []
        self.robot_proxy_client_dict = {}
        self.sensor_name_list = []
        self.sensor_proxy_client_dict = {}
        self.subtask_name_list = []
        self.subtask_proxy_client_dict = {}

        self.message_proxy = SimpleMessageProxy()

    def loadInstructions(self, instruction_list):
        self.instruction_list = instruction_list
        self.instruction_index = 0

    def addRobot(self, robot_name):
        self.robot_name_list.append(robot_name)
        self.robot_proxy_client_dict[robot_name] = CommandProxyClient("{}_supervisor".format(robot_name))
        self.robot_proxy_client_dict[robot_name].registerDoneCallback(self.robot_done_callback)

    def addSensor(self, sensor_name):
        self.sensor_name_list.append(sensor_name)
        self.sensor_proxy_client_dict[sensor_name] = CommandProxyClient("{}_supervisor".format(sensor_name))
        self.sensor_proxy_client_dict[sensor_name].registerDoneCallback(self.sensor_done_callback)

    def addSubTask(self, subtask_name):
        self.subtask_name_list.append(subtask_name)
        self.subtask_proxy_client_dict[subtask_name] = CommandProxyClient("{}_supervisor".format(subtask_name))
        self.subtask_proxy_client_dict[subtask_name].registerDoneCallback(self.subtask_done_callback)

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ CALLBACKS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def robot_done_callback(self, response_command):
        if response_command:
            sent_message = response_command.getSentMessage()
            sent_command = sent_message.getCommand()
            print(response_command.response_data)

            # ➤ Trajectory Commands
            if sent_message.getData("command_type") == "trajectory":
                try:
                    if response_command.response_data["trajectory_done"]:
                        print("trajectory OK   (dist={})".format(response_command.response_data["q_target_distance"]))
                        self.last_action_success = True
                    else:
                        q_dist = response_command.response_data["q_target_distance"]
                        print("trajectory FAIL   (dist={})".format(response_command.response_data["q_target_distance"]))
                        self.last_action_success = False
                except:
                    self.last_action_success = False
                    # print("trajectory FAIL   (error {})".format(response_command.response_data["error"]))

            # ➤ Setting Commands
            else:
                self.last_action_success = (response_command.status == CommandMessage.COMMAND_STATUS_OK)
                sent_command = response_command.getSentMessage().getCommand()
        else:
            sent_command = "NONE"
            self.last_action_success = False
        succ = "\033[91m" + "\033[92m" * self.last_action_success + "\033[1m\033[4m{}\033[0m".format(self.last_action_success)
        print("robot: \033[1m\033[4m\033[94m{}\033[0m >>> SUCCESS = {}".format(sent_command, succ))
        self.next()  # TODO andiamo sempre avanti nell'esecuzione così!!!! Dovremmo fare una condizione su "self.last_action_success and self.keep_tray_trajectory"

    def sensor_done_callback(self, response_command):
        if response_command:
            self.last_action_success = (response_command.status == CommandMessage.COMMAND_STATUS_OK)
            sent_command = response_command.getSentMessage().getCommand()
        else:
            sent_command = "NONE"
            self.last_action_success = False
        succ = "\033[91m" + "\033[92m" * self.last_action_success + "\033[1m\033[4m{}\033[0m".format(self.last_action_success)
        print("sensor: \033[1m\033[4m\033[94m{}\033[0m >>> SUCCESS = {}".format(sent_command, succ))
        self.next()

    def subtask_done_callback(self, response_command):
        if response_command:
            self.last_action_success = (response_command.status == CommandMessage.COMMAND_STATUS_OK)
            sent_command = response_command.getSentMessage().getCommand()
        else:
            sent_command = "NONE"
            self.last_action_success = False
        succ = "\033[91m" + "\033[92m" * self.last_action_success + "\033[1m\033[4m{}\033[0m".format(self.last_action_success)
        print("subtask: \033[1m\033[4m\033[94m{}\033[0m >>> SUCCESS = {}".format(sent_command, succ))
        self.next()

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PRIVATE METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def nextInstruction(self):

        if self.instruction_index >= len(self.instruction_list):
            if self.repete:
                self.instruction_index = 0
            else:
                self.idle()
                print("\n\n\n\n\n\033[91m\033[1m\033[4m{}\033[0m\n\n\n\n\n".format("TASK ENDED!!"))
                return

        cmd = self.instruction_list[self.instruction_index]
        self.instruction_index += 1

        # Lablel Instruction
        if cmd.startswith("___") and cmd.endswith("___"):
            self.next()
            return

        # Instruction Structure:  <subject> ␣ <command> ␣ [data]
        cmd_fiends = cmd.split(" ")

        subject = cmd_fiends[0]
        action = cmd_fiends[1]
        data = {}

        # ▇▇▇▇▇▇▇▇▇▇ Machine Instructions ▇▇▇▇▇▇▇▇▇▇
        if subject in self.robot_name_list + self.sensor_name_list + self.subtask_name_list:

            #⬢⬢⬢⬢⬢➤ Trajectory Commands
            if action == "gotoshape":
                shape_name = cmd_fiends[2]
                data = {"shape_name": shape_name, "command_type": "trajectory"}
            elif action in ["gototf", "movetotf", "jumptotf"]:
                tf_name = cmd_fiends[2]
                tool_name = cmd_fiends[3]
                data = {"tf_name": tf_name, "tool_name": tool_name, "command_type": "trajectory"}

            #⬢⬢⬢⬢⬢➤ Setting Commands
            # Command Structure: <data_key>:::<data_value>
            elif cmd.find(":::") != -1:
                data_key = cmd_fiends[2].split(":::")[0]
                try:  # if data field is JSON
                    data_value = json.loads(cmd.split(":::")[1])
                except:
                    data_value = cmd_fiends[2].split(":::")[1]
                data = {data_key: data_value, "command_type": "setting"}

                if action == "settool":
                    data = data_value

            try:
                self._send_command(supervisor_id=subject,
                                   command=action,
                                   data=data)
            except:
                Logger.error("Command not recognized: {}".format(cmd))

        # ▇▇▇▇▇▇▇▇▇▇ System Instructions ▇▇▇▇▇▇▇▇▇▇
        elif subject == "system":
            if action == "sleep":
                print("sleeping...")
                sec = float(cmd_fiends[2])
                time.sleep(sec)
                self.next()
            elif action == "condition":
                condition_action = cmd_fiends[2].split(":::")[0]
                label = cmd_fiends[2].split(":::")[1]
                if condition_action == "jumptrue" and self.last_action_success:
                    self.instruction_index = self.instruction_list.index(label)
                elif condition_action == "jumpfalse" and not self.last_action_success:
                    self.instruction_index = self.instruction_list.index(label)
                elif condition_action == "jumpalways":
                    self.instruction_index = self.instruction_list.index(label)
                self.next()
            elif action == "set":
                param = cmd_fiends[2].split(":::")[0]
                setting = cmd_fiends[2].split(":::")[1]
                if param == "repete":
                    self.repete = not (setting.lower() == "false")
                    print("repete={}".format(self.repete))
                elif param == "robotfault":
                    self.keep_tray_trajectory = not (setting == "tryagain")
                    print("robot fault->{}".format(self.keep_tray_trajectory))
                self.next()
        else:
            Logger.error("Subject not recognized: {}".format(subject))

    def _send_command(self, supervisor_id, command, data={}):
        Logger.warning("Sending {} {} to {}".format(command, data, supervisor_id))
        message = SimpleMessage(receiver="{}_supervisor".format(supervisor_id),
                                command=command)
        for k in data.keys():
            message.setData(k, data[k])

        if supervisor_id in self.sensor_name_list:
            self.active_command = self.sensor_proxy_client_dict[supervisor_id].sendCommand(message.toString())
            self.action()
        elif supervisor_id in self.robot_name_list:
            self.active_command = self.robot_proxy_client_dict[supervisor_id].sendCommand(message.toString())
            self.action()
        elif supervisor_id in self.subtask_name_list:
            self.active_command = self.subtask_proxy_client_dict[supervisor_id].sendCommand(message.toString())
            self.action()
        else:
            self.alarm()

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ STATES ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ IDLE ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ IDLE: enter
    def on_enter_idle(self):
        Logger.log("State:  IDLE")
        # self.next()

    # ➤ ➤ ➤ ➤ ➤ IDLE: loop
    def on_loop_idle(self):
        pass
    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ NEXT ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ NEXT: enter
    def on_enter_next(self):
        Logger.log("State:  NEXT")
        self.nextInstruction()

    # ➤ ➤ ➤ ➤ ➤ NEXT: loop
    def on_loop_next(self):
        pass
    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ ACTION ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ ACTION: enter
    def on_enter_action(self):
        Logger.log("State:  ACTION")

    # ➤ ➤ ➤ ➤ ➤ ACTION: loop
    def on_loop_action(self):
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ ALARM ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ ALARM: enter
    def on_enter_alarm(self):
        Logger.error("State:  ALARM")

    # ➤ ➤ ➤ ➤ ➤ ALARM: loop
    def on_loop_alarm(self):
        pass


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    MANAGER SM CLASS    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class TaskManagerSM(object):
    def __init__(self, task_name):
        self.task_name = task_name

        # CREATE STATE MACHINE
        self.managerSFM = SFMachineTaskManagerDefinition(
            self.task_name)
        self.sfm = machines.SFMachine(name="sfm_" + self.task_name + "_manager",
                                      model=self.managerSFM)

        # DEFINE SM STATES
        self.sfm.addState("start")
        self.sfm.addState("idle")
        self.sfm.addState("next")
        self.sfm.addState("action")
        self.sfm.addState("alarm")

        # DEFINE SM TRANSITIONS
        # start ...
        self.sfm.addTransition("idle", "start", "idle")
        self.sfm.addTransition("next", "start", "next")
        # idle ...
        self.sfm.addTransition("idle", "idle", "idle")
        self.sfm.addTransition("alarm", "idle", "alarm")
        self.sfm.addTransition("next", "idle", "next")
        self.sfm.addTransition("action", "idle", "action")
        # next ...
        self.sfm.addTransition("next", "next", "next")
        self.sfm.addTransition("alarm", "next", "alarm")
        self.sfm.addTransition("action", "next", "action")
        self.sfm.addTransition("idle", "next", "idle")
        # alarm ...
        self.sfm.addTransition("idle", "alarm", "idle")
        self.sfm.addTransition("action", "alarm", "action")
        self.sfm.addTransition("next", "alarm", "next")
        self.sfm.addTransition("alarm", "alarm", "alarm")
        # action ...
        self.sfm.addTransition("idle", "action", "idle")
        self.sfm.addTransition("next", "action", "next")
        self.sfm.addTransition("action", "action", "action")

        self.sfm.create()
        self.sfm.set_state("start")
        Logger.log("\n\n ************* SFM ready to start ***********")

    def start(self, robot_list, sensor_list, instruction_list, subtask_list):
        Logger.log("\n\n ************* SFM start ***********")
        self.managerSFM.loadInstructions(instruction_list)
        for robot_name in robot_list:
            self.managerSFM.addRobot(robot_name)
        for sensor_name in sensor_list:
            self.managerSFM.addSensor(sensor_name)
        for subtask_name in subtask_list:
            self.managerSFM.addSubTask(subtask_name)
        self.sfm.getModel().next()

    def stepForward(self):
        self.sfm.loop()
