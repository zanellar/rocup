#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
from rocup.taskmanager.task_manager_state_machine import TaskManagerSM
from rocup.param.global_parameters import Parameters


if __name__ == '__main__':
    node = RosNode("grasp_task")
    node.setupParameter("hz", 50)
    node.setHz(node.getParameter("hz"))

    Parameters.change(obj="comau_smart_six",
                      param="TRAJECTORY_TIME",
                      value=15)

    grasp_closure = {
        "clip": (19, 39),
        "screwdriver": (23, 43),
        "pendrive": (17, 37),
        "box_brown": (17, 37),
        "box_yellow": (31, 51),
        "battery_black": (25, 45),
        "battery_green": (25, 45),
        "artifact_orange": (50, 65),
        "artifact_white": (25, 45),
        "artifact_black": (35, 55),
        "artifact_metal": (20, 40),
        "glue": (10, 45)
    }

    target = node.setupParameter("target", "")


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PARAMETERS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    task_name = "grasp_task"

    manipulator = "comau_smart_six"
    robot_list = [manipulator]

    sensor_list = []

    target_tf = "acquire_target"
    gripper = "gripper"
    subtask_list = [target_tf,
                    gripper]

    CLOSE_DIST = grasp_closure[target][0]  # 19
    OPEN_DIST = min(65, grasp_closure[target][0] + 25)

    CLOSE_DIST = str(CLOSE_DIST)
    OPEN_DIST = str(OPEN_DIST)

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ INSTRUCTIONS LIST ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    instruction_list = [
        "system set repete:::False",
        target_tf + " acquire",
        "system sleep 2",
        gripper + " " + OPEN_DIST,
        manipulator + " movetotf target_tf_1 gripper",   # moves over the obj
        manipulator + " movetotf target_tf_0 gripper",     # goes down to grasp it
        gripper + " " + CLOSE_DIST,
        "system sleep 3",
        manipulator + " movetotf target_tf_1 gripper",     # goes up straight
        manipulator + " gotoshape shape_box",
        gripper + " " + OPEN_DIST,
        "system sleep 2",
        manipulator + " movetotf tf_storage_table camera"

    ]


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    tskm = TaskManagerSM(task_name)

    tskm.start(robot_list, sensor_list, instruction_list, subtask_list)
    try:
        while node.isActive():
            tskm.stepForward()
            node.tick()
    except rospy.ROSInterruptException:
        pass
