#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rocup.param.global_parameters import Parameters
from rocup.robotcontrol.robot_direct_motion import DirectCommander

if __name__ == '__main__':
    robot_name = Parameters.get("GRASSHOPPER_NAME")
    rospy.init_node('{}_direct_motion'.format(robot_name))
    node_rate = 10
    rate = rospy.Rate(node_rate)

    cmd = DirectCommander(robot_name)
    try:
        while not rospy.is_shutdown():
            cmd.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
