#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import tf_conversions
from tf_conversions import posemath
from superros.logger import Logger
import superros.transformations as transformations
from rocup.utils.devices import ForceSensor, Joystick
from rocup.robotmotion.robot_motion_action import RobotMotionClient
from rocup.msg import *
from rocup.msg import RobotMotionResult, RobotMotionFeedback
from rocup.msg import RobotFollow
from rocup.param.global_parameters import Parameters
from rocup.storage.mongo import MessageStorage
import rocup.sfm.machines as machines
from rocup.proxy.target_follower_proxy import TargetFollowerProxy
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy
from rocup.robotsupervisor.robot_supervisor_state_machine import SupervisorSM
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
import PyKDL
import copy
import math
import tf
from rocup.param.global_parameters import Parameters


if __name__ == '__main__':
    robot_name = Parameters.get("BONMET_NAME")
    rospy.init_node('{}_supervisor_node'.format(robot_name))
    node_rate = 30
    rate = rospy.Rate(node_rate)

    bonmet_supervisor = SupervisorSM(robot_name)
    bonmet_supervisor .start()

    try:
        while not rospy.is_shutdown():
            bonmet_supervisor .stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
