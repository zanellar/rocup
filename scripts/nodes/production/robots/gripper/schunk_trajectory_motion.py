#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
from rocup.robotmotion.robot_motion_action import RobotMotionServer

from rocup.param.global_parameters import Parameters

if __name__ == '__main__':
    robot_name = Parameters.get("SCHUNK_NAME")
    rospy.init_node('{}_trajectory_motion'.format(robot_name))
    node_rate = 50
    rate = rospy.Rate(node_rate)

    schunk_action_server = RobotMotionServer(robot_name)

    try:
        while not rospy.is_shutdown():
            schunk_action_server.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
