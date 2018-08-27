#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
import scipy
import PyKDL
import tf

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from superros.logger import Logger
import superros.transformations as transformations
from superros.transformations import TwistToKDLVector, TwistFormKDLVector
from rocup.utils.devices import Joystick
from rocup.param.global_parameters import Parameters
from rocup.proxy.target_follower_proxy import TargetFollowerProxy
from rocup.proxy.command_proxy import CommandProxyClient, CommandMessage, CommandProxyServer
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy

from rocup.robotcontrol.controllers.other_controllers import *

WRIST_FT_SENSOR_POSITION = Parameters.get("WRIST_FT_SENSOR_POSITION_WRT_EEF")


class DirectCommander(object):

    def __init__(self, robot_name, controllers_dict=None):
        self.robot_name = robot_name
        self.current_tf = None
        self.target_tf = None
        self.tf_target_name = "follow_target"
        self.active = False
        self.command_success_flg = False

        # target follower
        self.target_follower = TargetFollowerProxy(self.robot_name)

        # alarm
        self.alarm_proxy = AlarmProxy(self.robot_name)
        self.alarm_proxy.registerAlarmCallback(self.alarm_callback)
        self.alarm_proxy.registerResetCallback(self.alarm_reset_callback)

        # command message
        self.message_proxy = SimpleMessageProxy(
            "{}_inner_message".format(self.robot_name))
        self.message_proxy.register(self.command_callback)

        # command action
        self.command_server_last_message = None
        self.command_server = CommandProxyServer(
            "{}_direct_commander".format(self.robot_name))
        self.command_server.registerCallback(
            self.command_action_callback)

        # tf
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Joystick Contorol
        joy_sub = rospy.Subscriber('/joy', Joy,
                                   self.joy_callback, queue_size=1)
        self.joystick = Joystick(translation_mag=0.0005,
                                 rotation_mag=0.0008)
        self.joystick.registerCallback(self.joystick_key_callback)
        self.joystick_flg = False    # Disable by default

        # Closed-Loop Control
        if controllers_dict is None or controllers_dict == {}:
            controllers_dict = {"none": NeutralController()}

        self.controllers_dict = controllers_dict
        self.active_controllers = {}
        self.active_controllers["none"] = NeutralController()
        Logger.warning("READY!!!")

        # Feedback
        self.feedback_data = {}

        # Force
        rospy.Subscriber('/atift', Twist,
                         self.force_feedback_callback, queue_size=1)

        # Tactile
        rospy.Subscriber('/tactile',  Float64MultiArray,
                         self.tactile_feedback_callback, queue_size=1)

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ CALLBACKS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    # TODO problema: OGNI VOLTA CHE SI AGGIUNGE UN FEEDBACK OCCORRE CREARE UNA CALLBACK!!!!
    def force_feedback_callback(self, msg):
        msg = self._getForceOnTool(msg)
        self.feedback_data["atift"] = msg

    def tactile_feedback_callback(self, msg):
        self.feedback_data["tactile"] = msg

    def joy_callback(self, msg):
        self.joystick.update(msg)

    def joystick_key_callback(self, down, up):
        # if self.joystick.KEY_BACK in down:
        #     Logger.log(" ALARM: CLEAR ")
        #     self.target_tf = self.current_tf
        #     self.alarm_proxy.resetAlarm()
        #     self.target_follower.resetAlarm()
        if self.joystick.KEY_START in down:
            Logger.log(" USER ALARM: ON ")
            self.alarm_proxy.setAlarm()
            self.target_follower.setAlarm()
        if self.joystick.KEY_LOG in down:
            self.target_tf = self.current_tf

    def command_action_callback(self, cmd):
        self.command_server_last_message = cmd
        cmd_msg = cmd.getSentMessage()
        self.command_callback(cmd_msg)

        time.sleep(0.1)
        self._send_command_result(self.command_success_flg)

    def command_callback(self, msg):
        try:
            self.command_success_flg = True
            if msg.isValid():
                receiver = msg.getReceiver()
                if msg.getReceiver() == "{}_direct_commander".format(self.robot_name):
                    command = msg.getCommand()

                    #⬢⬢⬢⬢⬢➤ ON
                    if command == "_on_":
                        Logger.log(
                            " ******** Direct Commander: ENABLE ******** ")
                        self._reset()
                        self._removeController()
                        self.active = True

                    #⬢⬢⬢⬢⬢➤ OFF
                    elif command == "_off_":
                        Logger.log(
                            " ******** Direct Commander: DISABLE ******** ")
                        self.current_tf = None
                        self._reset()
                        self._removeController()
                        self.active = False

                    #⬢⬢⬢⬢⬢➤ RESET
                    elif command == "_reset_":
                        Logger.log(" ******** RESET ******** ")
                        self._reset()

                    #⬢⬢⬢⬢⬢➤ JOYSTICK
                    elif command == "_joyon_":
                        Logger.log("  Joystick: ENABLE  ")
                        self.joystick_flg = True

                    elif command == "_joyoff_":
                        Logger.log("  Joystick: DISABLE  ")
                        self.joystick_flg = False

                    #⬢⬢⬢⬢⬢➤ SELECT CONTROLLER
                    elif command.startswith("controllerselect"):
                        ctrl_id_list = msg.getData("id")
                        Logger.log(
                            " Controller Selection: {}".format(ctrl_id_list))
                        for ctrl_id in ctrl_id_list:
                            if ctrl_id == "none":
                                Logger.log(" Neutral Controller ")
                                self._removeController()
                            else:
                                try:
                                    self._addController(ctrl_id)
                                except:
                                    self.command_success_flg = False
                                    Logger.error(
                                        "Wrong Controller ID: {}".format(ctrl_id))

                    #⬢⬢⬢⬢⬢➤ CONTROLLER PARAMETERS
                    elif command.startswith("controllerparameters"):
                        parameters = msg.getData("parameters")
                        Logger.log(" Parameters Update: {} ".format(parameters))

                        for ctrl_id in parameters.keys():
                            if ctrl_id in self.active_controllers.keys():
                                self.active_controllers[ctrl_id].setParameters(parameters[ctrl_id])
                            else:
                                self.command_success_flg = False
                                Logger.error(
                                    "Wrong Controller ID: {}".format(ctrl_id))

                    #⬢⬢⬢⬢⬢➤ REMOVE CONTROLLER
                    elif command.startswith("controllerdisable"):
                        Logger.log(" Removing Controller ")
                        ctrl_id_list = msg.getData("id")
                        for ctrl_id in ctrl_id_list:
                            if ctrl_id == "_all_":
                                self._reset()
                                self._removeController()
                            elif ctrl_id in self.active_controllers.keys():
                                self._reset(ctrl_id)
                                self._removeController(ctrl_id)
                            else:
                                self.command_success_flg = False
                                Logger.error(
                                    "Wrong Controller ID: {}".format(ctrl_id))

                    # ⬢⬢⬢⬢⬢➤ START CONTROLLERS
                    elif command.startswith("controllerstart"):
                        Logger.log(" Controllers Start {}".format(
                            self.active_controllers.keys()))
                        input_data = msg.getData("input_data")
                        time.sleep(0.1)
                        for ctrl_id in input_data.keys():
                            if ctrl_id in self.active_controllers.keys():
                                self.active_controllers[ctrl_id].start(
                                    input_data[ctrl_id])
                            else:
                                self.command_success_flg = False
                                Logger.error(
                                    "Wrong Controller ID: {}".format(ctrl_id))
                    else:
                        self.command_success_flg = False
                        Logger.warning("INVALID COMMAND: {}".format(command))

            else:
                receiver = msg.getReceiver()
                if msg.getReceiver() == "{}_direct_commander".format(self.robot_name):
                    self.command_success_flg = False
        except Exception as e:
            self.command_success_flg = False
            print(e)

        Logger.log(" \nActive Controllers: {}\n".format(
            self.active_controllers.keys()))

    def alarm_callback(self, alarm_info):
        if alarm_info != self.alarm_proxy.NONE_ALARM:
            Logger.error(" !!!!!!!!  ALARM  !!!!!!!! ")
            self.target_tf = self.current_tf
            self.active = False

    def alarm_reset_callback(self):
        Logger.warning(" ********  ALARM CLEAR  ******** ")
        self.target_tf = self.current_tf
        self._reset()

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PRIVATE METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def _send_command_result(self, success):
        if self.command_server_last_message:
            Logger.warning("Command result:{}".format(success))
            if success:
                self.command_server.resolveCommand(
                    self.command_server_last_message)
            else:
                self.command_server.rejectCommand(
                    self.command_server_last_message)
            self.command_server_last_message = None

    def _removeController(self, ctrl_id=None):
        if ctrl_id is None:
            self.active_controllers = {}
            self.active_controllers["none"] = NeutralController()
        else:
            if ctrl_id in self.active_controllers.keys():
                self.active_controllers.pop(ctrl_id)
                if self.active_controllers == {}:
                    self.active_controllers["none"] = NeutralController()
            else:
                Logger.error(
                    "Trying to remove a deactivate controller: {}".format(ctrl_id))

    def _addController(self, ctrl_id=None):
        if ctrl_id:
            self.active_controllers[ctrl_id] = self.controllers_dict[ctrl_id]
            if "none" in self.active_controllers.keys():
                self._removeController("none")
        else:
            Logger.error(
                "Trying to activate an unlisted controller: {}".format(ctrl_id))

    def _reset(self, ctrlid=None):
        self.target_tf = self.current_tf
        if ctrlid is None:
            for ctrl_id in self.active_controllers.keys():
                self.active_controllers[ctrl_id].reset()
        else:
            self.active_controllers[ctrlid].reset()

    def _getForceOnTool(self, twist_on_sensor):
        linear_force_on_sensor = PyKDL.Frame(TwistToKDLVector(twist_on_sensor))
        tr_link6_fs = PyKDL.Frame(PyKDL.Vector(WRIST_FT_SENSOR_POSITION[0], WRIST_FT_SENSOR_POSITION[1], WRIST_FT_SENSOR_POSITION[2]))
        tr_link6_fs.M = tr_link6_fs.M.RotX(-scipy.pi)
        tr_tool_link6 = transformations.retrieveTransform(self.listener, self.robot_name + "/tool",
                                                          self.robot_name + "/link6", none_error=True,
                                                          time=rospy.Time(0), print_error=True)
        # TODO aggiungere una tf eef (in comau coincide con link6 in shunk con finger1 (me media tra i 2))
        try:
            tr_tool_fs = tr_tool_link6 * tr_link6_fs
            linear_force_on_tool = tr_tool_fs * linear_force_on_sensor
            twist_on_tool = TwistFormKDLVector(linear_force_on_tool.p)
            return twist_on_tool
        except Exception as e:
            print(e)

    def _derivateLinearForce(self, twist):
        dT = Twist()
        dT.linear.x = self.force_calculator.derivate(twist.linear.x)
        dT.linear.y = self.force_calculator.derivate(twist.linear.y)
        dT.linear.z = self.force_calculator.derivate(twist.linear.z)
        np.savetxt(self.file_handler,
                   [[time.time(),
                     dT.linear.x,
                     dT.linear.y,
                     dT.linear.z]],
                   fmt='%.18e %.18e %.18e %.18e',
                   delimiter=' ')
        return dT

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PUBLIC METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def stepForward(self):
        try:
            if self.active:
                self.current_tf = transformations.retrieveTransform(self.listener, self.robot_name + "/base_link",      # TODO aggiungere base_link in shunck
                                                                    self.robot_name + "/tool", none_error=True,
                                                                    time=rospy.Time(0), print_error=True)

                if self.current_tf is not None:
                    if self.target_tf is None:
                        self.target_tf = self.current_tf

                    #➤ Joystick
                    if self.joystick_flg:
                        self.target_tf = self.joystick.transformT(
                            self.target_tf)

                    #➤ Control Action
                    else:
                        self.feedback_data["current_tf"] = self.current_tf
                        self.feedback_data["target_tf"] = self.target_tf
                        for ctrl_id in self.active_controllers.keys():
                            self.target_tf = self.active_controllers[ctrl_id].output(self.feedback_data)

                    #######  Publish and Set Target  #######
                    transformations.broadcastTransform(self.br, self.target_tf, self.tf_target_name,
                                                       self.robot_name + "/base_link", time=rospy.Time.now())

                    dist = 1
                    # diff_tf = self.current_tf.Inverse() * self.target_tf
                    # transformations.broadcastTransform(self.br, diff_tf, "diff_tf",
                    #                                 self.robot_name + "/base_link", time = rospy.Time.now())

                    # dist_lin, distr_ang=transformations.frameDistance(diff_tf,
                    #                                                   PyKDL.Frame())
                    # distr_ang=np.linalg.norm(distr_ang)
                    # dist_lin=np.linalg.norm(
                    #     transformations.KDLVectorToNumpyArray(dist_lin))
                    # dist=distr_ang + dist_lin
                    # print dist
                    if dist > 0.0003 or self.joystick_flg:
                        self.target_follower.setTarget(target=self.target_tf,
                                                       target_type=TargetFollowerProxy.TYPE_POSE,
                                                       target_source=TargetFollowerProxy.SOURCE_DIRECT)
                else:
                    Logger.error("tf not ready!!")

            else:
                pass
        except Exception as e:
            Logger.warning(e)
            pass
