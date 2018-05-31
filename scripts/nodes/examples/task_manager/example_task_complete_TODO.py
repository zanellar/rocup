#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf
from superros.comm import RosNode
from superros.logger import Logger
from rocup.param.global_parameters import Parameters
from rocup.taskmanager.task_manager_state_machine import TaskManagerSM
import superros.transformations as transformations
import json


def getControlInputs(node, ctrl_id):
    if ctrl_id == "wire_insertion":
        hole_tf = None
        while not hole_tf:
            try:
                hole_tf = node.retrieveTransform(frame_id="tf_storage_hole",
                                                 parent_frame_id="comau_smart_six/base_link",
                                                 time=-1)
                hole_tf = transformations.KDLtoTf(hole_tf)
            except Exception as e:
                print("Waiting for 'tf_storage_hole'...")

        print("'tf_storage_hole' FOUND!!!")
        return {"hole_tf": hole_tf,  "wire_angle": 0}
    else:
        return None


if __name__ == '__main__':

    #⬢⬢⬢⬢⬢➤ NODE
    node = RosNode("wires_task")
    node.setupParameter("hz", 50)
    node.setHz(node.getParameter("hz"))

    task_name = "wires_task_comau"

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ Parameters ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    #⬢⬢⬢⬢⬢➤ Robots
    comau_name = Parameters.get("COMAU_NAME")
    gripper_name = Parameters.get("SCHUNK_NAME")
    robot_list = [comau_name, gripper_name]

    #⬢⬢⬢⬢⬢➤ Sensors
    tactile_name = "tactile"
    sensor_list = [tactile_name]

    #⬢⬢⬢⬢⬢➤ Subtasks
    insertion_task_name = "insertion_task"
    tool_correction_task_name = "tool_correction_task"
    wire_tf_filter_task_name = "tf_filter"
    subtask_list = [insertion_task_name, tool_correction_task_name, wire_tf_filter_task_name]

    #⬢⬢⬢⬢⬢➤ Controllers
    insertion_control_params = {
        "step_size": 0.0001,
        "force_p_gain": 1000.0,
        "force_threshold": 0.3,
        "threshold": [0.15, 2, 2]
    }

    insertion_control_inputs = getControlInputs(node, "wire_insertion")

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ INSTRUCTION LIST ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    instruction_list = [

        # --------------------------------
        # ----- initial configuration ----
        # --------------------------------

        comau_name + " gototf tf_storage_normal gripper",
        gripper_name + " gotoshape shape_open_max",
        tactile_name + " reset",

        # # --------------------------------
        # # ---------- wire grasp ----------
        # # --------------------------------

        # # fixed and known (wire position)
        # comau_name + " gototf tf_grasp_approach_1 gripper",
        # comau_name + " gototf tf_storage_grasp gripper",
        # gripper_name + " gotoshape shape_close",
        # comau_name + " gototf tf_grasp_approach_1 gripper",

        # detected by vision (wire position)
        comau_name + " gototf tf_storage_wire_detection camera",
        "system sleep 10",
        wire_tf_filter_task_name + " clear tf_name:::terminal_tf",
        wire_tf_filter_task_name + " start tf_name:::terminal_tf",
        comau_name + " gototf tf_wire_2 gripper",
        comau_name + " gototf tf_wire_1 gripper",
        gripper_name + " gotoshape shape_close",

        # --------------------------------
        # ------- tool correction --------
        # --------------------------------

        comau_name + " gototf tf_storage_camera_front gripper",
        "system sleep 15",
        tool_correction_task_name + " correct tool_name:::gripper",
        comau_name + " gototf tf_storage_camera_front dynamic",

        comau_name + " gototf tf_hole_approach_1 dynamic",

        # --------------------------------
        # ----------- insertion ----------
        # --------------------------------
        tactile_name + " reset",
        comau_name + " direct active:::" + json.dumps(True),
        comau_name + " controllerdisable id:::" + json.dumps(["_all_"]),
        comau_name + " controllerselect id:::" + json.dumps(["wire_insertion"]),
        comau_name + " controllerparameters parameters:::" + json.dumps({"wire_insertion": insertion_control_params}),
        "system sleep 0.1",
        comau_name + " controllerstart input_data:::" + json.dumps({"wire_insertion": insertion_control_inputs}),
        "system sleep 0.1",
        insertion_task_name + " waitend condition:::fuzzy_insertion",
        "system condition jumpfalse:::___insertion_fail___",
        comau_name + " direct active:::" + json.dumps(False),
        "system sleep 15",  # <---- avvitatore
        gripper_name + " gotoshape shape_open",

        "___insertion_fail___",
        comau_name + " gototf tf_hole_approach_1 dynamic"

    ]

    ###############################################################################################
    ###############################################################################################
    ###############################################################################################

    tskm = TaskManagerSM(task_name)

    tskm.start(robot_list, sensor_list, instruction_list, subtask_list)
    try:
        while node.isActive():
            tskm.stepForward()
            node.tick()
    except rospy.ROSInterruptException:
        pass
