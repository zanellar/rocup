#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import PyKDL
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import superros.transformations as transformations
import rocup.robots.controllers as controllers
from superros.logger import Logger

from rocup.srv import IKService, IKServiceResponse


""" GLOBAL PARAMETERS """

ROBOT_MARKET_LIBRARY = {

    #⬢⬢⬢⬢⬢➤ COMAU
    "comau_smart_six": {
        "robot_name": "comau_smart_six",
        "joint_names": [
            'base_to_link1',
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_link6',
        ],
        "link_names": [
            'base_link',
            'link1',
            'link2',
            'link3',
            'link4',
            'link5',
            'link6'
        ],
        "tools": {
            "default": [0, 0, 0, 0, 0, 0, 1]
        },
        "parameters": {
            "joints_tollerance": 0.0005,
            "repeatability": 0.00006
        },
        "auto_ik_service": True
    },

    #⬢⬢⬢⬢⬢➤ GRASSHOPPER
    "grasshopper": {
        "robot_name": "grasshopper",
        "joint_names": [
            'base_to_link1',
            'link1_to_link2',
            'link2_to_centerpoint',
            'centerpoint_to_finger_r',
            'centerpoint_to_finger_l',
            'base_to_driver'
        ],
        "link_names": [
            'base_link',
            'link1',
            'link2',
            'centerpoint',
            'finger_r',
            'finger_l',
            'driver'
        ],
        "tools": {
            "default": [0, 0, 0, 0, 0, 0, 1]
        },
        "parameters": {
            "joints_tollerance": 0.02,
            "repeatability": 0.00005
        },
        "auto_ik_service": True
    },

    #⬢⬢⬢⬢⬢➤ SCHUNK
    "schunk_pg70": {
        "robot_name": "schunk_pg70",
        "joint_names": [
            'pg70_finger1_joint',
            'pg70_finger2_joint'
        ],
        "link_names": [
            'schunk_pg70_base',
            'schunk_pg70_finger1',
            'schunk_pg70_finger2'
        ],
        "tools": {
            "default": [0, 0, 0, 0, 0, 0, 1]
        },
        "joint_topics": {
            "joint_command_topic": "/schunk_pg70/joint_command",
            "joint_state_topic": "/schunk_pg70/joint_states"
        },
        "parameters": {
            "joints_tollerance": 0.0005,
            "repeatability": 0.00005
        },
        "auto_ik_service": True
    },

    #⬢⬢⬢⬢⬢➤ BONMET
    "bonmetc60": {
        "robot_name": "bonmetc60",
        "joint_names": [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ],
        "link_names": [
            'base_link',
            'link1',
            'link2',
            'link3',
            'link4',
            'link5',
            'link6'
        ],
        "tools": {
            "default": [0, 0, 0, 0, 0, 0, 1]
        },
        "parameters": {
            "joints_tollerance": 0.008,
            "repeatability": 0.00006
        },
        "auto_ik_service": True
    }
}

ROBOT_MARKET_SHAPE_LIBRARIES = {

    "comau_smart_six": [
        {"name": "home", "q": [0, 0, 0, 0, 0, 0]}
    ],
    "bonmetc60": [
        {"name": "home", "q": [0, 0, 0, 0, -1.57, 0]}
    ],
    "grasshopper": [
        {"name": "home", "q": [0, 0, 0, 0]},
        {"name": "open", "q": [0, 0, 0.01, 0.01]},
        {"name": "close", "q": [0, 0, 0.001, 0.001]}
    ],
    "schunk_pg70": [
        {"name": "home", "q": [0.0, 0.0]},
        {"name": "open", "q": [0.01, 0.01]},
        {"name": "close", "q": [0.001, 0.001]}
    ]
}


class RobotMarket(object):

    @staticmethod
    def createRobotByName(name, tf_listener=None):
        cfg = ROBOT_MARKET_LIBRARY.get(name)
        if cfg != None:
            robot = controllers.Robot(configuration=cfg,
                                      tf_listener=tf_listener)

            shapes = ROBOT_MARKET_SHAPE_LIBRARIES.get(name)
            if shapes:
                for s in shapes:
                    robot_shape = controllers.RobotShape(s["name"], s["q"])
                    robot.addRobotShape(robot_shape)
            return robot
        else:
            Logger.error("No Robot with name '{}'".format(name))
            return None
