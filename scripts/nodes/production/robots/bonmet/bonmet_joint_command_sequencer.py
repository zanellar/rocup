#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy

from superros.logger import Logger
from rocup.param.global_parameters import Parameters
from superros.comm import RosNode
from sensor_msgs.msg import JointState


def command_callback(msg):
    global joints
    joints = msg.position


if __name__ == '__main__':
    robot_name = Parameters.get("BONMET_NAME")
    node = RosNode('joint_command_sequencer'.format(robot_name))
    node_rate = Parameters.get(obj=robot_name, param="NODE_FREQUENCY")
    node.setupParameter("hz", node_rate)

    node.createSubscriber("/bonmetc60/joint_command", JointState, command_callback)
    pub = node.createPublisher("/bonmetc60/joint_command_seq", JointState)

    joints = [0, 0, 0, 0, 0, 0]
    joints_msg = JointState()
    while node.isActive():
        joints_msg.position = joints
        pub.publish(joints_msg)
        node.tick()
