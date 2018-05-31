#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy

from rocup.msg import RobotFollow
from superros.logger import Logger
from rocup.param.global_parameters import Parameters
from rocup.robots.controllers import RobotStatus
from rocup.robotcontrol.robot_direct_motion import DirectCommander
from rocup.proxy.target_follower_proxy import TargetFollowerProxy
from rocup.proxy.alarm_proxy import AlarmProxy
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy


from rocup.robotcontrol.controllers.other_controllers import *
from rocup.robotcontrol.controllers.force_controllers import *
from rocup.robotcontrol.controllers.tactile_controllers import *
from rocup.robotcontrol.controllers.ml_controllers import *

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import PyKDL
import copy
import math
import tf
from rocup.param.global_parameters import Parameters


if __name__ == '__main__':
    robot_name = Parameters.get("BONMET_NAME")
    rospy.init_node('{}_direct_motion'.format(robot_name))
    node_rate = 100   #cambiare e mettere parametro 
    rate = rospy.Rate(node_rate)
    
    controllers_dict = {
            "none": NeutralController(), 
            "tactile_spring": MLSpringTactileController(), 
            "tactile_dampedforward":  MLDampedForwardTactileController(),
            "wire_insertion":  WireInsertionController()
        }
        
    cmd = DirectCommander(robot_name, controllers_dict)
    try:
        while not rospy.is_shutdown():
            cmd.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
