#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy

from std_msgs.msg import *


import random
import numpy
from numpy import *
import numpy as np
import math
import pprint
import time
pp = pprint.PrettyPrinter(indent=4)
from superros.logger import Logger
import superros.transformations as transformations
from rocup.storage.mongo import MessageStorage
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import PyKDL
from PyKDL import Frame, Vector, Rotation
import threading

from std_msgs.msg import String


import tf
import cv2
import time

import json

from rospkg import RosPack


class SaveParams():

    def __init__(self):
        self.str_msg = String()
        self.message_database = MessageStorage()

    def save(self, input_name, data_dict):
        self.str_msg.data = json.dumps(data_dict)
        try:
            self.message_database.replace(input_name, self.str_msg)
        except Exception as e:
            print("Parameter {}, NOT saved\n {}".format(input_name, e))

    def get(self, input_name):
        try:
            json_string = self.message_database.searchByName(
                input_name,
                String,
                single=False
            )
            data = json.loads(json_string[0][0].data)
            return data
        except Exception as e:
            print e
            return None

    def delete(self, input_name):
        try:
            self.message_database.deleteByName(input_name, String)
        except Exception as e:
            print e
