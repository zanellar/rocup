#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
import superros.transformations as transformations
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
import numpy as np
import PyKDL

# DA SPOSTARE
import message_filters
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import JointState, Image, CompressedImage, JointState
from geometry_msgs.msg import Twist, Pose
import datetime
import time


def topicsCallback(*args):
    print(args)


timestamp = str(datetime.datetime.now()).replace(
    '.', '_').replace(":", "_").replace(' ', '_')
#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("topic_export")

node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))
node.setupParameter("output_path", "/tmp/topic_export/")
node.setupParameter("dataset_name", timestamp)
node.setupParameter("time_window", 9999999.0)
node.setupParameter("default_image_format", 'jpg')
node.setupParameter("default_image_padding", 5)

print("Dataset: {}".format(timestamp))


topic_map = {
    # '/matrix_tf_sequencer/moving_flag': eval("Int32"),
    # '/matrix_tf_sequencer/tool_pose': eval("Pose"),
    '/comau_smart_six/tool': eval("Pose"),
    # '/usb_cam/image_raw/compressed': eval('CompressedImage')
    '/usb_cam_1/image_raw/compressed': eval('CompressedImage')
    #'/darknet_rgb_detector/predictions': eval('Float64MultiArray')
    #'/camera/depth/image_raw': eval('Image')
}

container_map = {}
callback_map = {}
callback_managers = {}
data_map = {}
times = []


def pushNewData():
    for topic, data in container_map.iteritems():

        data_map[topic].append(data['value'])
        print(topic, len(data_map[topic]))
    times.append(node.getCurrentTimeInSecs())


class DataManager():
    TIMINGS_NAME = "timings"

    def __init__(self, output_path):
        self.data_matrix = {}
        self.data_types = {}
        self.times = []
        self.output_path = output_path

    @staticmethod
    def converMsgInData(msg, data_type):

        print("{}".format(data_type))
        if data_type == PyKDL.Frame:
            return transformations.KDLtoNumpyVector(msg).reshape(7)
        if data_type == Twist:
            return np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ]).T
        if data_type == Pose:
            if msg is None:
                return np.zeros(7)
            else:
                return np.array([
                    msg.position.x,
                    msg.position.y,
                    msg.position.z,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ]).T
        if data_type == JointState:
            if msg is None:
                msg = JointState()
            arr = np.array([])
            if len(msg.position) > 0:
                arr = np.asarray(msg.position)
            else:
                arr = np.array([0])
            if len(msg.velocity) > 0:
                arr = np.hstack((arr, np.asarray(msg.velocity)))
            else:
                arr = np.hstack((arr, np.array([0])))
            return arr.T
        if data_type == Float64MultiArray:
            return np.array([msg.data]).T
        if data_type == Int32:
            if msg is None:
                return np.zeros(1)
            else:
                return np.array([msg.data])

        if data_type == CompressedImage:
            np_arr = np.fromstring(msg.data, np.uint8)

            try:
                return cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            except:
                return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def pushData(self, topic_name, time, msg, data_type):
        if topic_name not in self.data_matrix:
            self.data_matrix[topic_name] = []

        self.data_types[topic_name] = data_type
        raw_data = DataManager.converMsgInData(msg, data_type)
        self.data_matrix[topic_name].append(raw_data)

    def pushTime(self, time):
        self.times.append(time)

    def setTimes(self, times):
        self.data_matrix[DataManager.TIMINGS_NAME] = (
            np.array(times) * 1000.0).astype(int)
        self.data_types[DataManager.TIMINGS_NAME] = Float64MultiArray

    @staticmethod
    def purgeTopicName(topic_name):
        if topic_name.startswith('/'):
            topic_name = topic_name[1:]
        return topic_name.replace('/', '_')

    def saveData(self):
        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)

        for topic, data in self.data_matrix.iteritems():
            purged_name = DataManager.purgeTopicName(topic)
            filename = os.path.join(self.output_path, purged_name)

            if self.data_types[topic] == CompressedImage or self.data_types[topic] == Image:
                if not os.path.exists(filename):
                    os.makedirs(filename)

                image_format = node.getParameter("default_image_format")
                image_number_padding = node.getParameter(
                    "default_image_padding")

                image_paths = []
                for i in range(0, len(self.data_matrix[topic])):
                    image_filename = os.path.join(
                        filename, 'rgb_{}.{}'.format(str(i).zfill(image_number_padding), image_format))
                    cv2.imwrite(image_filename, self.data_matrix[topic][i])
                    image_paths.append(image_filename)
                np.savetxt(filename + ".txt", image_paths, fmt="%s")
            else:
                np.savetxt(filename + ".txt",
                           np.array(self.data_matrix[topic]))


class CallbackManager():
    def __init__(self, topic):
        self.topic = str(topic)

    def callback(self, msg):
        container_map[self.topic]['value'] = msg

    @staticmethod
    def createACallback(name):

        callback_managers[name] = CallbackManager(name)
        return callback_managers[name].callback


for t, msg_type in topic_map.iteritems():
    topic = str(t)
    container_map[topic] = {'value': None}
    data_map[topic] = []

    callback_map[topic] = CallbackManager.createACallback(topic)
    node.createSubscriber(topic, msg_type, callback_map[topic])


while node.isActive():
    if node.getElapsedTimeInSecs() >= node.getParameter("time_window"):
        break
    try:
        # time.sleep(0.1)
        pushNewData()
        print("Sync time ready!")

        node.tick()
    except KeyboardInterrupt:
        print("Interrupt")


CV_BRIDGE = CvBridge()
output_path = os.path.join(
    node.getParameter("output_path"),
    node.getParameter("dataset_name")
)
data_manager = DataManager(output_path)

print("Saving")
print("Times", len(times))
for topic, data in container_map.iteritems():
    for i in range(0, len(data_map[topic])):
        time = times[i]
        data_manager.pushData(
            topic, time, data_map[topic][i], topic_map[topic])


data_manager.setTimes(times)
data_manager.saveData()
