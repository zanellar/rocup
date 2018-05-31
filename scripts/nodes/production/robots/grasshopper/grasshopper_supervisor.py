#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rocup.param.global_parameters import Parameters
from rocup.robotsupervisor.robot_supervisor_state_machine import SupervisorSM

if __name__ == '__main__':
    robot_name = Parameters.get("GRASSHOPPER_NAME")
    rospy.init_node('{}_supervisor_node'.format(robot_name))
    node_rate = 30
    rate = rospy.Rate(node_rate)

    grasshopper_supervisor = SupervisorSM(robot_name)
    grasshopper_supervisor.start()

    try:
        while not rospy.is_shutdown():
            grasshopper_supervisor.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
