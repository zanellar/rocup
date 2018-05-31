#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
import sys
from rocup.ui.pyqt import PyQtWindow
from superros.comm import RosNode
from rocup.storage.save_shape import SaveShape
from rocup.storage.save_parameters import SaveParams
from rocup.storage.tf_storage import TfStorage
from superros.logger import Logger
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
import superros.transformations as transformations
from superros.transformations import FrameVectorFromKDL, FrameVectorToKDL, ListToKDLVector, KDLVectorToList
from rocup.storage.mongo import MessageStorage
from rocup.proxy.command_proxy import CommandProxyClient
from rocup.stubs.tf_manager_stub import TfManagerStub
from sensor_msgs.msg import JointState
import PyKDL
import rospy
import math
import numpy
import time

import json
from numpy import *
import tf

from tf.msg import tfMessage
from std_msgs.msg import String, Float64MultiArray

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("custom_window_test")
node.setupParameter("robot_name", "comau_smart_six")  # bonmetc60
robot_name = node.getParameter("robot_name")


class CustomWindow(PyQtWindow):

    def __init__(self, uifile, node):
        super(CustomWindow, self).__init__(uifile=uifile)

        self.robot_name = robot_name

        ############# Variable Initialization #############
        self.available_tfs_map = {}
        self.available_shapes_map = {}
        self.available_ctrlpar_map = {}
        self.wire_z_offset = 0
        self.wire_x_offset = 0
        self.wire_angle = 0
        self.direct_control_active = False
        self.controller_input = {}
        self.controller_params = {}
        self.selected_tf = ""
        self.selected_tool = "gripper"
        self.selected_shape = ""
        self.selected_axis = [1, 0, 0]
        self.selected_controller = "none"
        self.command_sequence_index = 0
        self.seq_run_flg = False
        self.active_command = None

        ################
        self.listener = tf.TransformListener()

        self.node = node
        self.node.createSubscriber("/tf", tfMessage, self.tf_callback)

        self.robot_proxy_client = CommandProxyClient(
            "{}_supervisor".format(self.robot_name))
        self.robot_proxy_client.registerDoneCallback(self.robot_done_callback)
        self.robot_message_proxy = SimpleMessageProxy()
        self.inner_message_proxy = SimpleMessageProxy(
            "{}_inner_message".format(self.robot_name))
        self.gripper_pub = self.node.createPublisher(
            "/schunk_pg70/joint_setpoint", JointState)
        # self.filter_pub = self.node.createPublisher(
        #     "/simple_message_stream/common", String)

        ################
        self.tf_manager = TfManagerStub()
        self.param_saver = SaveParams()
        self.shape_saver = SaveShape(self.robot_name)
        self.alarm_proxy = AlarmProxy(self.robot_name)
        self.alarm_proxy.registerAlarmCallback(self.alarm_callback)
        self.alarm_proxy.registerResetCallback(self.alarm_reset_callback)
        self.message_database = MessageStorage()
        # self.command_proxy_client = CommandProxyClient(
        #     "{}_supervisor".format(self.robot_name))
        # self.command_proxy_client.registerDoneCallback(self.supervisor_done_callback)

        ############# Qt Objects ############
        self.alarm_set_button.setStyleSheet("background-color: green")
        self.direct_button.setStyleSheet("background-color: white")
        #---
        self.tool_box.currentIndexChanged.connect(self.tool_change)
        self.gripper_check_box.stateChanged.connect(self.gripper)
        self.joystick_check_box.stateChanged.connect(self.joystick)
        self.direct_button.clicked.connect(self.directMode)
        self.start_control_button.clicked.connect(self.startControl)
        self.clear_control_button.clicked.connect(self.clearControlList)
        self.select_control_button.clicked.connect(self.selectControl)
        self.update_controller_button.clicked.connect(
            self.setControlParameters)
        self.load_ctrlpar_button.clicked.connect(
            self.loadControlParameters)
        self.direct_reset_button.clicked.connect(self.directReset)
        self.update_button.clicked.connect(self.tfListUpdate)
        self.update_button_2.clicked.connect(self.shapeListUpdate)
        self.filter_button.clicked.connect(self.startFilter)
        self.clear_filter_button.clicked.connect(self.clearFilter)
        self.save_shape_button.clicked.connect(self.saveShape)
        self.save_controller_button.clicked.connect(self.saveController)
        self.update_button_3.clicked.connect(self.ctrlParamsListUpdate)
        self.delete_shape_button.clicked.connect(self.deleteShape)
        self.save_tf_button.clicked.connect(self.saveTf)
        self.alarm_reset_button.clicked.connect(self.reset_alarm)
        self.alarm_set_button.clicked.connect(self.set_alarm)
        self.tf_filter.textChanged.connect(self.tfListUpdate)
        self.shape_filter.textChanged.connect(self.shapeListUpdate)
        self.send_command.clicked.connect(self.sendCommand)
        self.forward_button.clicked.connect(self.commandSequence)
        self.forward_button.clicked.connect(self.nextCommand)
        self.set_index_button.clicked.connect(self.setCommandSequenceIndex)
        self.reset_index_button.clicked.connect(self.resetCommandSequenceIndex)

        self.tfListUpdate()

    def setCommandSequenceIndex(self):
        self.command_sequence_index = int(self.sequence_index_value.text())

    def resetCommandSequenceIndex(self):
        self.sequence_index_value.setText("0")
        self.command_sequence_index = -1
        self.nextCommand()

    def commandSequence(self, command_str):
        if command_str.startswith("FILTER"):
            self.selected_tf = command_str.split(" ")[1]
            self.clearFilter()
            self.startFilter()
        elif command_str.startswith("CLOSE_GRIPPER"):
            self.gripper_check_box.setChecked(True)
            self.gripper()
        elif command_str.startswith("OPEN_GRIPPER"):
            self.gripper_check_box.setChecked(False)
            self.gripper()
        else:
            self.current_command.setText(command_str)
            if self.link_to_send_check_box.isChecked():
                self.sendCommand()

    def sendCommand(self):
        print(self.current_command.text())
        cmd_str = str(self.current_command.text())
        command = str(self.command_type_box.currentText())
        if cmd_str.startswith("shape_"):
            simple_message = SimpleMessage(command=command,
                                           receiver="{}_supervisor".format(self.robot_name))
            shape_name = cmd_str.split(" ")[0]  # self.selected_shape
            simple_message.setData("shape_name", shape_name)
        else:
            simple_message = SimpleMessage(command=command,
                                           receiver="{}_supervisor".format(self.robot_name))
            tf_name = cmd_str.split(" ")[0]  # self.selected_tf
            tool_name = cmd_str.split(" ")[1]  # self.selected_tool
            simple_message.setData("tf_name", tf_name)
            simple_message.setData("tool_name", tool_name)
        print(simple_message.toString())
        self.active_command = self.robot_proxy_client.sendCommand(
            simple_message.toString())
        # self.robot_message_proxy.send(simple_message)
        self.direct_button.setStyleSheet("background-color: white")
        self.send_command.setStyleSheet("background-color: red")

    def robot_done_callback(self, command):
        sent_message = command.getSentMessage()
        if sent_message.getCommand().startswith("goto"):
            Logger.warning("trajectory DONE")
            try:
                source_done = command.response_data["trajectory_done"]
                if source_done:
                    print("trajectory OK   {}".format(
                        command.response_data["q_target_distance"]))
                    self.send_command.setStyleSheet("background-color: green")
                else:
                    print("trajectory FAIL   {}".format(
                        command.response_data["q_target_distance"]))
                    self.send_command.setStyleSheet("background-color: white")
            except Exception as e:
                print(e)
                self.send_command.setStyleSheet("background-color: red")

    def updateCommand(self, target_type):
        if target_type == "shape":
            command = "{}".format(self.selected_shape)
            self.current_command.setText(command)
        elif target_type == "tf":
            command = "{} {}".format(self.selected_tf,
                                     self.selected_tool)
            self.current_command.setText(command)
        else:
            return

    def updateList(self, commands):
        self.commands = commands
        self.command_sequence_index = 0
        model = QStandardItemModel(self.list)
        for c in commands:
            item = QStandardItem(str(c))
            model.appendRow(item)
        self.list.setModel(model)
        self.list.selectionModel().currentChanged.disconnect()
        self.list.selectionModel().currentChanged.connect(self.listSelectionChanged)
        self.setSelectedCommand(self.command_sequence_index)

    def setSelectedCommand(self, index):
        index = self.list.model().index(index, 0)
        self.list.setCurrentIndex(index)

    def nextCommand(self):
        self.command_sequence_index += 1
        self.command_sequence_index = self.command_sequence_index % len(
            self.commands)
        self.setSelectedCommand(self.command_sequence_index)

    def listSelectionChanged(self, v):
        print(v.data().toString())
        command = v.data().toString()
        self.current_command_text.setText(command)
        self.commandSequence(str(command))

    def directMode(self):
        simple_message = SimpleMessage(command="direct",
                                       receiver="{}_supervisor".format(self.robot_name))
        self.direct_control_active = not self.direct_control_active
        if self.direct_control_active:
            self.direct_button.setStyleSheet("background-color: green")
        else:
            self.direct_button.setStyleSheet("background-color: white")
        simple_message.setData("active", self.direct_control_active)
        self.robot_message_proxy.send(simple_message)

    def directReset(self):
        simple_message = SimpleMessage(command="directreset",
                                       receiver="{}_supervisor".format(self.robot_name))
        self.robot_message_proxy.send(simple_message)

    def set_alarm(self):
        self.alarm_set_button.setStyleSheet("background-color: red")
        self.enter_ready_to_reset = True
        self.alarm_proxy.setAlarm()

    def alarm_callback(self, alarm_info):
        if alarm_info != self.alarm_proxy.NONE_ALARM:
            self.alarm_set_button.setStyleSheet("background-color: red")
            self.enter_ready_to_reset = True
        elif alarm_info == self.alarm_proxy.NONE_ALARM and self.enter_ready_to_reset:
            self.alarm_set_button.setStyleSheet("background-color: orange")
            self.enter_ready_to_reset = False

    def alarm_reset_callback(self):
        self.alarm_set_button.setStyleSheet("background-color: green")

    def reset_alarm(self):
        self.alarm_set_button.setStyleSheet("background-color: green")
        self.enter_ready_to_reset = False
        cmd_str = "reset"
        simple_message = SimpleMessage(command=cmd_str,
                                       receiver="{}_supervisor".format(self.robot_name))
        self.robot_message_proxy.send(simple_message)

    def gripper(self):
        close_val = int(self.gripper_close_value.text())
        open_val = int(self.gripper_open_value.text())
        if self.gripper_check_box.isChecked() == True:
            self._setGripper(close_val)
        else:
            self._setGripper(open_val)

    def _setGripper(self, p, v=50, e=50):
        js = JointState()
        js.name = []
        js.position = [p]
        js.velocity = [v]
        js.effort = [e]
        self.gripper_pub.publish(js)

    def joystick(self):
        if self.joystick_check_box.isChecked() == True:
            cmd_str = "_joyon_"
        else:
            cmd_str = "_joyoff_"
        simple_message = SimpleMessage(command=cmd_str,
                                       receiver="{}_direct_commander".format(self.robot_name))
        self.inner_message_proxy.send(simple_message)
        print(cmd_str)


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    TRAJECTORY TAB    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def setSelectedTf(self, tf_name):
        self.selected_tf = str(tf_name)
        self.updateCommand("tf")

    def setSelectedShape(self, shape_name):
        self.selected_shape = shape_name
        self.updateCommand("shape")

    def tool_change(self, i):
        self.selected_tool = str(self.tool_box.currentText())
        simple_message = SimpleMessage(command="selecttool",
                                       receiver="{}_supervisor".format(self.robot_name))
        simple_message.setData("tool_name", str(self.selected_tool))
        print(simple_message.toString())
        self.robot_message_proxy.send(simple_message)

        self.updateCommand("tf")

    def tf_change(self, v):
        itms = self.tf_list.selectedIndexes()
        for it in itms:
            self.setSelectedTf(it.data().toString())
            break

    def shape_change(self, v):
        itms = self.shape_list.selectedIndexes()
        for it in itms:
            self.setSelectedShape(it.data().toString())
            break

    def set_new_tool(self, transf):
        simple_message = SimpleMessage(command="settool",
                                       receiver="{}_supervisor".format(self.robot_name))
        simple_message.setData("tool_name", str(self.selected_tool))
        simple_message.setData("new_tool_name", "dynamic")
        simple_message.setData("transformation", transf)
        print(simple_message.toString())
        self.robot_message_proxy.send(simple_message)

    def tf_callback(self, msg):
        self.available_tfs = []
        for tf in msg.transforms:
            if len(tf.child_frame_id) > 0:
                self.available_tfs_map[tf.child_frame_id] = tf

    def tfListUpdate(self):
        tf_list_model = QStandardItemModel(self.tf_list)
        tf_sorted = sorted(self.available_tfs_map.keys())
        filtered_str = str(self.tf_filter.text()).lower()
        for tf_name in tf_sorted:
            if filtered_str in tf_name.lower() or filtered_str == '':
                item = QStandardItem(tf_name)
                item.setEditable(False)
                tf_list_model.appendRow(item)
        self.tf_list.setModel(tf_list_model)
        self.tf_list.selectionModel().selectionChanged.connect(self.tf_change)
        self.tf_manager.update()

    def shapeListUpdate(self):
        shape_list_model = QStandardItemModel(self.shape_list)
        db_msg = self.message_database.searchByName("shape_",
                                                    JointState,
                                                    single=False)

        for i in range(0, len(db_msg)):
            name = db_msg[i][1]["name"]
            shape = db_msg[i][0].position
            self.available_shapes_map[name] = shape
        shape_sorted = sorted(self.available_shapes_map.keys())
        filtered_str = str(self.shape_filter.text()).lower()
        for shape_name in shape_sorted:
            if filtered_str in shape_name.lower() or filtered_str == '':
                item = QStandardItem(shape_name)
                item.setEditable(False)
                shape_list_model.appendRow(item)
        self.shape_list.setModel(shape_list_model)
        self.shape_list.selectionModel().selectionChanged.connect(self.shape_change)

    def startFilter(self):
        msg_str = String()
        tf_name = self.selected_tf
        msg = SimpleMessage(receiver="tf_filter_supervisor", command="start")
        msg.setData("slot_size", 200)
        msg.setData("tf_name", tf_name)
        self.robot_message_proxy.send(msg)

    def clearFilter(self):
        msg_str = String()
        tf_name = self.selected_tf
        msg = SimpleMessage(receiver="tf_filter_supervisor", command="clear")
        msg.setData("tf_name", tf_name)
        self.robot_message_proxy.send(msg)

    def saveShape(self):
        self.shape_saver.save("shape_" + str(self.obj_save_name_value.text()))

    def deleteShape(self):
        self.shape_saver.delete(
            "shape_" + str(self.obj_save_name_value.text()))
        print("delete " + str(self.obj_save_name_value.text()))

    def saveTf(self):
        current_tool_tf = node.retrieveTransform(
            self.robot_name + "/base_link",
            self.robot_name + "/tool",
            -1
        )
        if current_tool_tf:
            self.tf_manager.save(tf_name=self.robot_name + "/tool",
                                 tf_parent=self.robot_name + "/base_link",
                                 saving_name=str(self.obj_save_name_value.text()))
            time.sleep(0.5)
            self.tf_manager.update()
            time.sleep(0.5)
            self.tf_manager.startPublish()
            print("OK! Saved:{}".format(str(self.obj_save_name_value.text())))

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    CONTROLLERS TAB    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def ctrlParamsListUpdate(self):
        list_model = QStandardItemModel(self.ctrlpar_list)
        db_msg = self.message_database.searchByName("ctrl_",
                                                    String,
                                                    single=False)

        for i in range(0, len(db_msg)):
            name = db_msg[i][1]["name"]
            data = json.loads(db_msg[i][0].data)
            self.available_ctrlpar_map[name] = data
        ctrlpar_sorted = sorted(self.available_ctrlpar_map.keys())
        for ctrlpar_name in ctrlpar_sorted:
            item = QStandardItem(ctrlpar_name)
            item.setEditable(False)
            list_model.appendRow(item)
        self.ctrlpar_list.setModel(list_model)
        self.ctrlpar_list.selectionModel().selectionChanged.connect(self.ctrlpar_change)

    def ctrlpar_change(self):
        itms = self.ctrlpar_list.selectedIndexes()
        for it in itms:
            self.selected_ctrlpar.setText(it.data().toString())
            break

    def saveController(self):
        self.setControlParameters(action="save")
        print self.controller_params
        self.param_saver.save("ctrl_" + str(self.obj_save_name_value.text()),
                              self.controller_params)

    def clearControlList(self):
        simple_message = SimpleMessage(command="controllerdisable",
                                       receiver="{}_supervisor".format(self.robot_name))
        simple_message.setData("id",
                               ["_all_"])
        self.robot_message_proxy.send(simple_message)

    def startControl(self):
        controller_input = {}
        #➤ force
        if "force_spring" in self._getControllersList():
            controller_input["force_spring"] = {}
        if "force_dampedforward"in self._getControllersList():
            controller_input["force_dampedforward"] = {
                "axis": self.selected_axis}
        #➤ tactile
        if "tactile_spring" in self._getControllersList():
            controller_input["tactile_spring"] = {}
        if "tactile_dampedforward" in self._getControllersList():
            controller_input["tactile_dampedforward"] = {}
        if "wire_insertion" in self._getControllersList():
            hole_tf = None
            while not hole_tf:
                try:
                    hole_tf = node.retrieveTransform(frame_id="tf_storage_hole",
                                                     parent_frame_id=self.robot_name + "/base_link",
                                                     time=-1)
                    hole_tf = transformations.KDLtoTf(hole_tf)
                except:
                    print("Waiting for 'tf_storage_hole'...")

            if hole_tf:
                controller_input["wire_insertion"] = {"hole_tf": hole_tf,
                                                      "wire_angle": self.wire_angle}
            else:
                print("\n\n\n\nHOLE TF NOT FOUND\n\n\n\n")
        if len(self._getControllersList()) == 0:
            controller_input["none"] = {}

        simple_message = SimpleMessage(command="controllerstart",
                                       receiver="{}_supervisor".format(self.robot_name))
        simple_message.setData("input_data",
                               controller_input)
        self.robot_message_proxy.send(simple_message)
        print(simple_message.toString())

    def selectControl(self):
        simple_message = SimpleMessage(command="controllerselect",
                                       receiver="{}_supervisor".format(self.robot_name))

        if len(self._getControllersList()) == 0:
            simple_message.setData("id", ["none"])
        else:
            simple_message.setData("id",
                                   self._getControllersList())
        self.robot_message_proxy.send(simple_message)
        print(simple_message.toString())

    def updateControlParam(self):
        simple_message = SimpleMessage(command="controllerparameters",
                                       receiver="{}_supervisor".format(self.robot_name))
        simple_message.setData("parameters", self.controller_params)
        self.robot_message_proxy.send(simple_message)
        print(simple_message.toString())

    def loadControlParameters(self):
        self.controller_params = self.available_ctrlpar_map[str(
            self.selected_ctrlpar.text())]
        print self.controller_params
        self.updateControlParam()

    def setControlParameters(self, action="load"):

        spring_force_params = {"translation_mag": float(self.translation_mag_value.text()),
                               "rotation_mag": float(self.rotation_mag_value.text()),
                               "thresholds": eval(str(self.force_thresholds_value.text()))}

        damp_force_params = {"velocity": float(self.velocity_value.text()),
                             "damp_force_threshold": float(self.damp_threshold_value.text()),
                             "damp_magnitude": float(self.damp_mag_value.text())}

        tactile_spring_params = {
            "threshold": [float(self.tactile_threshold_value_0.text()), float(self.tactile_threshold_value_1.text()), float(self.tactile_threshold_value_2.text())],
            "linear_gain": [float(self.tactile_lin_gain_value_0.text()), float(self.tactile_lin_gain_value_1.text()), float(self.tactile_lin_gain_value_2.text())],
            "angular_gain": [float(self.tactile_ang_gain_value_0.text()), float(self.tactile_ang_gain_value_1.text()), float(self.tactile_ang_gain_value_2.text())],
            "angular_action": self.angular_action_check_box.isChecked(),
            "linear_action": self.linear_action_check_box.isChecked()
        }
        tactile_damp_params = {
            "step_size": float(self.step_size_value.text()),
            "direction_gain": float(self.direction_gain_value.text()),
            "regulation_angular_action": self.angular_reg_action_check_box.isChecked(),
            "regulation_linear_action": self.linear_reg_action_check_box.isChecked(),
            "linear_regulation_gain": [float(self.lin_reg_gain_value_0.text()), float(self.lin_reg_gain_value_1.text()), float(self.lin_reg_gain_value_2.text())],
            "angular_regulation_gain": [float(self.ang_reg_gain_value_0.text()), float(self.ang_reg_gain_value_1.text()), float(self.ang_reg_gain_value_2.text())],
            "regulation_threshold": [float(self.regulation_threshold_value_0.text()), float(self.regulation_threshold_value_1.text()), float(self.regulation_threshold_value_2.text())],
            "global_gain": float(self.global_gain_value.text()),
            "direction_correction": self.correction_action_check_box.isChecked(),
            "direction_compensation": self.compensation_action_check_box.isChecked()
        }
        wire_insertion_params = {
            "step_size": float(self.in_step_size_value.text()),
            "force_p_gain": float(self.in_force_pgain_value.text()),
            "force_threshold": float(self.in_force_threshold_value.text()),
            "regulation_p_gain": [float(self.in_regulation_pgain_value_0.text()), float(self.in_regulation_pgain_value_1.text()), float(self.in_regulation_pgain_value_2.text())],
            "regulation_i_gain": [float(self.in_regulation_igain_value_0.text()), float(self.in_regulation_igain_value_1.text()), float(self.in_regulation_igain_value_2.text())],
            "regulation_i_size": [float(self.in_regulation_isize_value_0.text()), float(self.in_regulation_isize_value_1.text()), float(self.in_regulation_isize_value_2.text())],
            "threshold": [float(self.in_regulation_threshold_value_0.text()), float(self.in_regulation_threshold_value_1.text()), float(self.in_regulation_threshold_value_2.text())],
            "force_projection":  self.force_projection_check_box.isChecked(),
            "position_ball_radious":  float(self.position_ball_radious_value.text()),
            "position_ball_offset":  float(self.position_ball_offset_value.text()),
            "position_scaling_gain":  float(self.in_position_scaling_gain.text()),
            "position_scaling_limits":  [float(self.in_position_scaling_limmin.text()), float(self.in_position_scaling_limmax.text())]
        }

        self.controller_params = {}
        #➤ force
        if "force_spring" in self._getControllersList():
            self.controller_params["force_spring"] = spring_force_params
        if "force_dampedforward"in self._getControllersList():
            self.controller_params["force_dampedforward"] = damp_force_params
        #➤ tactile
        if "tactile_spring"in self._getControllersList():
            self.controller_params["tactile_spring"] = tactile_spring_params
        if "tactile_dampedforward"in self._getControllersList():
            self.controller_params["tactile_dampedforward"] = tactile_damp_params
        if "wire_insertion"in self._getControllersList():
            self.controller_params["wire_insertion"] = wire_insertion_params

        print action
        if action == "save":
            pass
        else:
            self.updateControlParam()

    def _getControllersList(self):
        selected_controllers = []
        #➤ force
        if self.spring_force_ctrl_box.isChecked():
            selected_controllers.append("force_spring")
        return selected_controllers

    def getAxis(self, axis_name):
        sign = int("{}1".format(axis_name[0]))
        axis = axis_name[1]
        if axis == "x":
            return [sign, 0, 0]
        elif axis == "y":
            return [0, sign, 0]
        elif axis == "z":
            return [0, 0, sign]
        else:
            return [0, 0, 0]


ui_file = node.getFileInPackage(
    "rocup", "data/ui/robot_gui.ui")
w = CustomWindow(uifile=ui_file, node=node)


# def loadCommand(self):
#     commands = []
#     n_list = int(1)
#     with open('command_list_{}.txt'.format(n_list)) as inputfile:
#         for line in inputfile:
#             commands.append(line)

# commands = ["shape_table",
#             "OPEN_GRIPPER",
#             "TACTILE_RESET",
#             "shape_camera_on_table",
#             "FILTER Terminal0",
#             "Terminal0_approach_1 gripper",
#             "Terminal0_approach_0 gripper",
#             "CLOSE_GRIPPER",
#             "Terminal0_approach_1 gripper",
#             "tf_storage_ground_camera gripper",
#             "TOOL_CORRECTION gripper",
#             "tf_storage_ground_camera dynamic",
#             "component_approach_1 dynamic",
#             "tf_storage_component dynamic",
#             "component_approach_1 dynamic"
#             ]

commands = ["OPEN_GRIPPER",
            "TACTILE_RESET",
            "tf_wire_2 gripper",
            "tf_wire_3 gripper",
            "tf_wire_1 gripper",
            "FILTER Terminal0",
            "Terminal0_approach_1 gripper",
            "Terminal0_approach_0 gripper",
            "CLOSE_GRIPPER",
            "tf_wire_4 gripper"
            ]


w.updateList(commands)

w.run()
