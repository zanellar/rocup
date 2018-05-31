#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
from std_msgs.msg import String
import numpy
from numpy import *
from rocup.robotmotion.robot_motion_action import RobotMotionServer
from rocup.msg import *
from rocup.msg import RobotMotionResult, RobotMotionFeedback, RobotMotionAction

from superros.logger import Logger

from rocup.param.global_parameters import Parameters

if __name__ == '__main__':
    robot_name = Parameters.get("COMAU_NAME")
    rospy.init_node('{}_trajectory_motion'.format(robot_name))
    node_rate = Parameters.get(obj=robot_name, param="NODE_FREQUENCY")
    rate = rospy.Rate(node_rate)

    comau_action_server = RobotMotionServer(robot_name)

    try:
        while not rospy.is_shutdown():
            comau_action_server.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
