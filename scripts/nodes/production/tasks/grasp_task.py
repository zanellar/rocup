#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
from rocup.taskmanager.task_manager_state_machine import TaskManagerSM


if __name__ == '__main__':
    node = RosNode("grasp_task")
    node.setupParameter("hz", 50)
    node.setHz(node.getParameter("hz"))

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PARAMETERS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    task_name = "grasp_task"

    manipulator = "comau_smart_six"
    robot_list = [manipulator]

    sensor_list = []

    target_tf = "acquire_target"
    gripper = "gripper"
    subtask_list = [target_tf,
                    gripper]

    CLOSE_DIST = "1"
    OPEN_DIST = "40"
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
        manipulator + " movetotf target_tf_1 gripper"     # goes up straight
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
