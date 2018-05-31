#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosnode
import PyKDL
import tf
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from superros.logger import Logger
import superros.transformations as transformations
from superros.transformations import FrameVectorFromKDL, FrameVectorToKDL
from rocup.robotmotion.robot_motion_action import RobotMotionClient
from rocup.storage.mongo import MessageStorage
import rocup.sfm.machines as machines
from rocup.proxy.target_follower_proxy import TargetFollowerProxy
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer
from rocup.param.global_error_list import GlobalErrorList
from rocup.param.global_parameters import Parameters
from rocup.robots.market import RobotMarket


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    STATE MACHINE DEFINITION    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class SFMachineRobotSupervisorDefinition(object):

    DIRECT_COMMANDER = 100
    TRAJECTORY_COMMANDER = 101

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.listener = tf.TransformListener()
        self.message_database = MessageStorage()

        # tool
        self.tools_list = Parameters.get(obj=self.robot_name, param="TOOLS")
        print self.tools_list
        self.new_tools = {}
        self.new_tools["dynamic"] = FrameVectorToKDL(self.tools_list["gripper"])

        # alarm
        self.alarm_proxy = AlarmProxy(self.robot_name)
        self.alarm_proxy.registerAlarmCallback(self.alarm_callback)

        # target follower proxy
        self.target_follower = TargetFollowerProxy(self.robot_name)

        # robot outer message (no feedback/response)
        self.outer_message_proxy = SimpleMessageProxy()
        self.outer_message_proxy.register(self.command_callback)

        # robot outer command
        self.command_server = CommandProxyServer(
            "{}_supervisor".format(self.robot_name))
        self.command_server.registerCallback(
            self.command_action_callback)
        self.command_server_last_message = None
        self.command_category = "unknown"

        # robot inner message
        self.inner_message_proxy = SimpleMessageProxy(
            "{}_inner_message".format(self.robot_name))

        # robot inner command
        self.robot_trajectory_client = RobotMotionClient(self.robot_name,
                                                         ext_callback=self.robot_trajectory_callback)
        self.robot_control_client = CommandProxyClient(
            "{}_direct_commander".format(self.robot_name))
        self.robot_control_client.registerDoneCallback(
            self.control_done_callback)
        # self.robot_control_client.registerFeedbackCallback(
        #     self.control_feedback_callback)   # TODO

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ CALLBACKS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def alarm_callback(self, alarm_info):
        if alarm_info != self.alarm_proxy.NONE_ALARM:
            Logger.error(" !!!!!!!!  ALARM  !!!!!!!! ")
            self.alarm()

    def robot_trajectory_callback(self, cb_type, msg):
        if cb_type == self.robot_trajectory_client.FEEDBACK:
            self.trajectory_sm_feedback = msg
            # self._send_trajectory_feedback(msg) # TODO
        elif cb_type == self.robot_trajectory_client.RESULT:
            self._send_trajectory_command_result(msg)
            if self.state != "alarm":
                self.idle()

    def control_done_callback(self, command):
        # print command
        if command:
            success = (command.status == CommandMessage.COMMAND_STATUS_OK)
            command = command.getSentMessage().getCommand()
        else:
            command = "NONE"
            success = False
        self._send_command_result(success)
        print("controller: {} >>> SUCCESS={}".format(command, success))

        # if self.state != "alarm":
        #     self.idle()

    def command_action_callback(self, cmd):
        self.command_server_last_message = cmd
        cmd_msg = cmd.getSentMessage()
        self.command_callback(cmd_msg)

    def command_callback(self, msg):
        if msg.isValid():
            if msg.getReceiver() == "{}_supervisor".format(self.robot_name):
                print msg.toString()
                command = msg.getCommand()
                try:
                    #⬢⬢⬢⬢⬢➤ RESET
                    if command == "reset":
                        self.command_category = "setting"
                        Logger.warning(" !!!!!!!! ALARM RESET  !!!!!!!! ")
                        self.alarm_proxy.resetAlarm()
                        self.target_follower.resetAlarm()
                        self._reject_input_command(GlobalErrorList.ROBOT_ALARM)
                        self.idle()

                    elif self.state != "alarm":
                        #⬢⬢⬢⬢⬢➤ SET TOOL
                        if command == "settool":    # change multiple data structure in a single multi-entry dictionary
                            self.command_category = "setting"
                            tool_name = msg.getData("tool_name")
                            new_tool_name = msg.getData("new_tool_name")
                            transf = msg.getData("transformation")
                            tool = self._getToolFromName(tool_name)
                            transf = FrameVectorToKDL(transf)
                            self.new_tools[new_tool_name] = tool * transf
                            print("Tools: {}".format(self.new_tools))
                            self._send_command_result(True)
                            self.idle()

                        #⬢⬢⬢⬢⬢➤ SELECT TOOL
                        elif command == "selecttool":
                            self.command_category = "setting"
                            tool_name = msg.getData("tool_name")
                            tool = self._getToolFromName(tool_name)
                            self._setTool(tool)
                            self._send_command_result(True)
                            self.idle()

                        #⬢⬢⬢⬢⬢➤ SET TRAJECTORY
                        elif command == "settrajectory":
                            self.command_category = "setting"
                            points = msg.getData("points")  # if it returns none, this wasn't the set parameter
                            time = msg.getData("time")
                            if points is not None:
                                print Parameters.get(obj=self.robot_name, param="TRAJECTORY_POINTS")
                                Parameters.change(obj=self.robot_name,
                                                  param="TRAJECTORY_POINTS",
                                                  value=int(points))
                                print Parameters.get(obj=self.robot_name, param="TRAJECTORY_POINTS")
                            elif time is not None:
                                Parameters.change(obj=self.robot_name,
                                                  param="TRAJECTORY_TIME",
                                                  value=int(points))
                            self._send_command_result(True)
                            self.idle()

                        #⬢⬢⬢⬢⬢➤ MOVE TO TF
                        elif command == "movetotf":
                            self.command_category = "trajectory"
                            tf_name = msg.getData("tf_name")
                            tool_name = msg.getData("tool_name")
                            self._toTf(tf_name, tool_name, mode=command)
                            print("Tf: \033[1m\033[4m\033[94m{}\033[0m".format(tf_name))
                            self.trajectory()

                        #⬢⬢⬢⬢⬢➤ JUMP TO TF
                        elif command == "jumptotf":
                            self.command_category = "trajectory"
                            tf_name = msg.getData("tf_name")
                            tool_name = msg.getData("tool_name")
                            self._toTf(tf_name, tool_name, mode=command)
                            print("Tf: \033[1m\033[4m\033[94m{}\033[0m".format(tf_name))
                            self.trajectory()

                        #⬢⬢⬢⬢⬢➤ GO TO TF
                        elif command == "gototf":
                            self.command_category = "trajectory"
                            tf_name = msg.getData("tf_name")
                            tool_name = msg.getData("tool_name")
                            self._toTf(tf_name, tool_name, mode=command)
                            print("Tf: \033[1m\033[4m\033[94m{}\033[0m".format(tf_name))
                            self.trajectory()

                        #⬢⬢⬢⬢⬢➤ GO TO JOINTS
                        elif command == "gotojoints":
                            self.command_category = "trajectory"
                            joints_str = msg.getData("joints_vector")
                            self._toJoints(joints_str)
                            print("Joints: \033[1m\033[4m\033[94m{}\033[0m".format(joints_str))
                            self.trajectory()

                        #⬢⬢⬢⬢⬢➤ GO TO SHAPE
                        elif command == "gotoshape":
                            self.command_category = "trajectory"
                            shape_name = msg.getData("shape_name", str)
                            self._toShape(shape_name)
                            print("Shape: \033[1m\033[4m\033[94m{}\033[0m".format(shape_name))
                            self.trajectory()

                        #⬢⬢⬢⬢⬢➤ RESET CONTROLLERS
                        elif command == "directreset":
                            self.command_category = "setting"
                            self._resetDirectCommander()

                        #⬢⬢⬢⬢⬢➤ ENABLE CONTROLLERS
                        elif command == "direct":
                            self.command_category = "setting"
                            active = msg.getData("active")
                            if active:
                                self._switchCommander(self.DIRECT_COMMANDER)
                                self.direct()
                            else:
                                self._switchCommander(
                                    self.TRAJECTORY_COMMANDER)
                                self.idle()

                        #⬢⬢⬢⬢⬢➤ START CONTROLLERS
                        elif command == "controllerstart":
                            self.command_category = "setting"
                            self._sendCommandActionToController(msg)
                            print("Inputs: {}".format(
                                msg.getData("input_data")))
                            self.controlling()

                        #⬢⬢⬢⬢⬢➤ UPDATE CONTROLLERS PARAMETERS
                        elif command == "controllerparameters":
                            self.command_category = "setting"
                            self._sendCommandActionToController(msg)
                            print("Params: {}".format(
                                msg.getData("parameters")))

                        #⬢⬢⬢⬢⬢➤ SELECT CONTROLLERS
                        elif command == "controllerselect":
                            self.command_category = "setting"
                            self._sendCommandActionToController(msg)
                            print("Controller selected: {}".format(
                                msg.getData("id")))

                        #⬢⬢⬢⬢⬢➤ DISABLE CONTROLLERS
                        elif command == "controllerdisable":
                            self.command_category = "setting"
                            self._sendCommandActionToController(msg)
                            print("Controllers removed: {}".format(
                                msg.getData("id")))
                        else:
                            self.command_category = "unknown"
                            Logger.error("INVALID input")
                            self._reject_input_command()
                except Exception as e:
                    Logger.error(e)
                    self._reject_input_command()

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PRIVATE METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def _getToolFromName(self, tool_name):
        tool_v = self.tools_list[tool_name]
        tool = FrameVectorToKDL(tool_v)
        return tool

    def _sendCommandActionToController(self, msg):
        msg.setReceiver("{}_direct_commander".format(self.robot_name))
        msg.setData("type", "force")
        self.robot_control_client.sendCommand(msg.toString())

    def _sendMessageToController(self, msg):
        msg.setReceiver("{}_direct_commander".format(self.robot_name))
        msg.setData("type", "force")
        self.inner_message_proxy.send(msg)

    def _resetDirectCommander(self):
        simple_message = SimpleMessage(command="_reset_",
                                       receiver="{}_direct_commander".format(self.robot_name))
        self._sendCommandActionToController(simple_message)

    def _setTool(self, tool):
        """ tool is a PyKDL frame"""
        self.target_follower.setTool(tool)

    def _toJoints(self, joint_str):
        self._switchCommander(self.TRAJECTORY_COMMANDER, expect_done_response=False)
        q_str = joint_str
        txt_cmd = "joints {}".format(q_str)
        self.robot_trajectory_client.generateNewGoal(txt_cmd)

    def _toShape(self, shape_name):
        self._switchCommander(self.TRAJECTORY_COMMANDER, expect_done_response=False)
        txt_cmd = self._getCommandFromShape(shape_name)
        self.robot_trajectory_client.generateNewGoal(txt_cmd)

    def _toTf(self, tf_name, tool_name, mode="goto"):
        self._switchCommander(self.TRAJECTORY_COMMANDER, expect_done_response=False)
        if tool_name.startswith("dynamic"):
            tool = self.new_tools[tool_name]
        else:
            tool = self._getToolFromName(tool_name)
        self._setTool(tool)
        txt_cmd = "{} {}".format(mode, tf_name)
        self.robot_trajectory_client.generateNewGoal(txt_cmd)

    def _getCommandFromShape(self, shape_name):
        db_msg = self.message_database.searchByName(shape_name,
                                                    JointState,
                                                    single=False)
        q_str = str(db_msg[0][0].position).split(
            "(")[1].split(")")[0].replace(" ", "")
        txt_cmd = "joints {}".format(q_str)
        return txt_cmd

    def _switchCommander(self, id, expect_done_response=True):
        if id == self.DIRECT_COMMANDER:
            simple_message = SimpleMessage(command="_on_",
                                           receiver="{}_direct_commander".format(self.robot_name))
            self.target_follower.setSource(
                TargetFollowerProxy.SOURCE_DIRECT)
        elif id == self.TRAJECTORY_COMMANDER:
            simple_message = SimpleMessage(command="_off_",
                                           receiver="{}_direct_commander".format(self.robot_name))
            self.target_follower.setSource(
                TargetFollowerProxy.SOURCE_TRAJECTORY)

        if expect_done_response:
            self._sendCommandActionToController(simple_message)
        else:
            self._sendMessageToController(simple_message)

    def _send_trajectory_command_result(self, msg):
        if self.command_server_last_message:
            if msg is None:
                error = GlobalErrorList.TRAJECTORY_ACTION_RESPONSE_NONETYPE
                Logger.error("Trajectory action response None-type")
                trajectory_done = False
                q_target_distance = -999
            else:
                error = msg.error
                q_target_distance = msg.q_target_distance
                trajectory_done = msg.success

            self.command_server_last_message.addResponseData("error",
                                                             error)
            self.command_server_last_message.addResponseData("q_target_distance",
                                                             q_target_distance)
            self.command_server_last_message.addResponseData("trajectory_done",
                                                             trajectory_done)

            self._send_command_result(trajectory_done)

    def _reject_input_command(self, error_type=GlobalErrorList.UNKNOWN_ERROR):
        if self.command_server_last_message:
            self.command_server_last_message.addResponseData("error",
                                                             error_type)
            if self.command_category == "trajectory":
                self._send_trajectory_command_result(None)
            else:
                self._send_command_result(False)

    def _send_command_result(self, success):
        if self.command_server_last_message:
            if success:
                self.command_server.resolveCommand(self.command_server_last_message)
            else:
                self.command_server.rejectCommand(self.command_server_last_message)
            self.command_server_last_message = None

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ STATES ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ IDLE ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ IDLE: enter
    def on_enter_idle(self):
        Logger.log("State:  IDLE")

    # ➤ ➤ ➤ ➤ ➤ IDLE: loop
    def on_loop_idle(self):
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ CONTROLLING ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ CONTROLLING: enter
    def on_enter_controlling(self):
        Logger.log("State:  CONTROLLING")

    # ➤ ➤ ➤ ➤ ➤ CONTROLLING: loop
    def on_loop_controlling(self):
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ DIRECT ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ DIRECT: enter
    def on_enter_direct(self):
        Logger.log("State:  DIRECT")

    # ➤ ➤ ➤ ➤ ➤ DIRECT: loop
    def on_loop_direct(self):
        if not rosnode.rosnode_ping("{}_direct_motion".format(self.robot_name)):
            self.alarm_proxy.setAlarm(AlarmProxy.NODE_DIED)
            self.alarm()
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ TRAJECTORY ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ TRAJECTORY: enter
    def on_enter_trajectory(self):
        Logger.log("State:  TRAJECTORY")
        # self._switchCommander(self.TRAJECTORY_COMMANDER)

    # ➤ ➤ ➤ ➤ ➤ TRAJECTORY: loop
    def on_loop_trajectory(self):
        if not rosnode.rosnode_ping("{}_trajectory_motion".format(self.robot_name)):
            self.alarm_proxy.setAlarm(AlarmProxy.NODE_DIED)
            self.alarm()
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ ALARM ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ ALARM: enter
    def on_enter_alarm(self):
        Logger.log("State:  ALARM")
        self._reject_input_command(GlobalErrorList.ROBOT_ALARM)

    # ➤ ➤ ➤ ➤ ➤ ALARM: loop
    def on_loop_alarm(self):
        self._reject_input_command(GlobalErrorList.ROBOT_ALARM)
        # pass


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    SUPERVISOR SM CLASS    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class SupervisorSM(object):
    def __init__(self, robot_name):
        self.robot_name = robot_name

        # CREATE STATE MACHINE
        self.supervisorSFM = SFMachineRobotSupervisorDefinition(
            self.robot_name)
        self.sfm = machines.SFMachine(name="sfm_" + self.robot_name + "_supervisor",
                                      model=self.supervisorSFM)

        # DEFINE SM STATES
        self.sfm.addState("start")
        self.sfm.addState("idle")
        self.sfm.addState("controlling")
        self.sfm.addState("trajectory")
        self.sfm.addState("alarm")
        self.sfm.addState("direct")

        # DEFINE SM TRANSITIONS
        # start ...
        self.sfm.addTransition("idle", "start", "idle")
        # idle ...
        self.sfm.addTransition("idle", "idle", "idle")
        self.sfm.addTransition("alarm", "idle", "alarm")
        self.sfm.addTransition("direct", "idle", "direct")
        self.sfm.addTransition("trajectory", "idle", "trajectory")
        # controlling ...
        self.sfm.addTransition("idle", "controlling", "idle")
        self.sfm.addTransition("controlling", "controlling", "controlling")
        self.sfm.addTransition("trajectory", "controlling", "trajectory")
        self.sfm.addTransition("alarm", "controlling", "alarm")
        # alarm ...
        self.sfm.addTransition("idle", "alarm", "idle")
        self.sfm.addTransition("alarm", "alarm", "alarm")
        # direct ...
        self.sfm.addTransition("idle", "direct", "idle")
        self.sfm.addTransition("direct", "direct", "direct")
        self.sfm.addTransition("controlling", "direct", "controlling")
        self.sfm.addTransition("trajectory", "direct", "trajectory")
        self.sfm.addTransition("alarm", "direct", "alarm")
        # trajectory ...
        self.sfm.addTransition("trajectory", "trajectory", "trajectory")
        self.sfm.addTransition("idle", "trajectory", "idle")
        self.sfm.addTransition("alarm", "trajectory", "alarm")

        self.sfm.create()
        self.sfm.set_state("start")
        Logger.log("\n\n ************* SFM ready to start ***********")

    def start(self):
        Logger.log("\n\n ************* SFM start ***********")
        self.sfm.getModel().idle()

    def stepForward(self):
        self.sfm.loop()
