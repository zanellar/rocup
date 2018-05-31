#!/usr/bin/env python

import rospy


class GlobalErrorList(object):
    NO_ERROR = 0
    UNKNOWN_ERROR = -1
    UNKNOWN_TRAJECTORY_ERROR = -2
    NO_TRAJECTORY_COMMAND = -3
    NO_TRAJECTORY = -4
    NO_START_MOVING = -5
    NO_END_TRAJECTORY = -6
    ROBOT_ALARM = -7
    TRAJECTORY_ACTION_RESPONSE_NONETYPE = -8
