#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
from rocup.taskmanager.task_manager_state_machine import TaskManagerSM


if __name__ == '__main__':
    node = RosNode("task_manager_template")
    node.setupParameter("hz", 50)
    node.setHz(node.getParameter("hz"))

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PARAMETERS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    task_name = "task_manager_template"

    robot_list = []
    sensor_list = []
    subtask_list = []

# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ INSTRUCTIONS LIST ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    instruction_list = [

        "system sleep 2"

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
