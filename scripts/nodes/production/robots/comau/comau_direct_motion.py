#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy

from rocup.param.global_parameters import Parameters
from rocup.robotcontrol.robot_direct_motion import DirectCommander

from rocup.robotcontrol.controllers.other_controllers import NeutralController, PIDCompassController
from rocup.robotcontrol.controllers.force_controllers import SpringForceController


if __name__ == '__main__':
    robot_name = Parameters.get("COMAU_NAME")
    rospy.init_node('{}_direct_motion'.format(robot_name))
    node_rate = 100
    rate = rospy.Rate(node_rate)

    controllers_dict = {
        "none": NeutralController(),
        "force_spring": SpringForceController(),
        "compass": PIDCompassController()
    }

    cmd = DirectCommander(robot_name, controllers_dict)
    try:
        while not rospy.is_shutdown():
            cmd.stepForward()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
