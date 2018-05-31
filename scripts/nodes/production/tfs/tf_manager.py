#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf
import superros.transformations as transformations
from superros.logger import Logger
from rocup.storage.tf_storage import TfStorage
from superros.comm import RosNode
from rocup.proxy.proxy_message import SimpleMessage, SimpleMessageProxy

import message_filters
import rospkg
import numpy as np
import math
import sys
import random

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("tf_manager")

node.setupParameter("hz", 30)
node.setupParameter("world_tf", "world")
node.setupParameter("target_tf", "target")
node.setupParameter("storage_prefix", "saved_")
node.setupParameter("max_save_iterations", 30)
node.setHz(node.getParameter("hz"))

#⬢⬢⬢⬢⬢➤ variables
message_proxy = SimpleMessageProxy()
tf_storage = TfStorage()
all_tfs = []
publishing = True


def command_cb(message):
    global current_tf_name, publishing, all_tfs
    if message.isValid():
        receiver = message.getReceiver()
        command = message.getCommand()
        if receiver == node.getName():
            print("New command received", message.toString())
            #⬢⬢⬢⬢⬢➤ UPDATE
            if command == 'update':
                all_tfs = tf_storage.allTfs()
                return
            #⬢⬢⬢⬢⬢➤ START PUBLISH
            if command == 'start_publish':
                publishing = True
                return
            #⬢⬢⬢⬢⬢➤ STOP PUBLISH
            if command == 'stop_publish':
                publishing = False
                return
            #⬢⬢⬢⬢⬢➤ DELETE_ALL
            if command == 'delete_all':
                tf_storage.deleteAll()
                all_tfs = []
                return
            #⬢⬢⬢⬢⬢➤ DELETE
            if command == 'delete':
                saving_name = message.getData("saving_name")
                tf_storage.removeFrameByName(saving_name)
                all_tfs = tf_storage.allTfs()
                return
            #⬢⬢⬢⬢⬢➤ SAVE
            if command == 'save':
                saving_name = message.getData("saving_name")
                tf_name = message.getData("tf_name")
                tf_parent = message.getData("tf_parent")
                iterations = node.getParameter("max_save_iterations")
                if tf_name == None:
                    Logger.error("'tf_name' must be not None")
                    return
                if tf_parent == None:
                    Logger.error("'tf_parent' must be not None")
                    return
                if saving_name == None:
                    Logger.error("'saving_name' must be not None")
                    return
                while iterations > 0:
                    frame = node.retrieveTransform(
                        tf_name,
                        tf_parent,
                        -1
                    )
                    if frame:
                        tf_storage.saveFrame(
                            frame,
                            saving_name,
                            tf_parent
                        )
                        print("Saving ", tf_name, tf_parent)
                        return
                Logger.error(
                    "Max iterations reached, retrieving '{}'".format(tf_name))


message_proxy.register(command_cb)

while node.isActive():
    if publishing:
        for tf in all_tfs:
            tf_parent = tf[1]["parent_tf"]
            tf_name = tf[1]["name"]
            tf_data = tf[0]
            # print("Publish", tf_parent, tf_name, tf_data)
            node.broadcastTransform(
                tf_data,
                tf_name,
                tf_parent,
                node.getCurrentTime()
            )
    node.tick()
