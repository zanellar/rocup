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
import collections
from std_msgs.msg import String


import tf
import cv2
import time


from rospkg import RosPack


class TfStorage():
    SOURCE_NAMESPACE = "TfStorage"
    DEFAULT_TF_PARENT = "world"

    def __init__(self, namespace="tf_storage"):
        self.namespace = namespace
        self.message_database = MessageStorage()

    def _getSavingName(self, input_name):
        prefix = ''
        saving_name = ''
        if self.namespace != '':
            prefix = self.namespace + '_'
        return prefix + input_name

    def loadFrame(self, input_name):
        name = self._getSavingName(input_name)
        result = self.message_database.searchByName(
            name,
            Pose,
            single=False
        )
        if len(result) > 0:
            result = result[0]
            return (transformations.PoseToKDL(result[0]), result[1])
        return None

    def allTfs(self):
        result = self.message_database.serachByCustomMeta(
            "source_namespace",
            TfStorage.SOURCE_NAMESPACE,
            Pose
        )
        results = []
        for r in result:
            results.append((transformations.PoseToKDL(r[0]), r[1]))
        return results

    def deleteAll(self):
        all_tfs = self.allTfs()
        for tf in all_tfs:
            try:
                self.message_database.deleteByID(str(tf[1]["_id"]))
            except:
                pass

    def removeFrameByName(self, name):
        result = self.loadFrame(name)
        if result:
            print("Deleteing", result)
            try:
                self.message_database.deleteByID(str(result[1]["_id"]))
            except:
                pass

    def saveFrame(self, frame, input_name, parent_tf=None):
        try:
            pose = transformations.KDLToPose(frame)
            name = self._getSavingName(input_name)
            if not parent_tf:
                parent_tf = TfStorage.DEFAULT_TF_PARENT

            #⬢⬢⬢⬢⬢➤ META
            meta = {
                'parent_tf': parent_tf,
                'source_namespace': TfStorage.SOURCE_NAMESPACE
            }
            self.message_database.replace(
                name,
                pose,
                meta=meta)
        except:
            pass
