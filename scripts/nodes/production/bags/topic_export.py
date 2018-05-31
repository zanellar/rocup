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
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState, Image, CompressedImage
from geometry_msgs.msg import Twist
import datetime
import copy


def topicsCallback(*args):
    print(args)


timestamp = str(datetime.datetime.now()).replace(
    '.', '_').replace(":", "_").replace(' ', '_')

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("topic_export")

node.setupParameter("hz", 120)
node.setHz(node.getParameter("hz"))
node.setupParameter("output_path", "/tmp/topic_export/")
node.setupParameter("dataset_name", timestamp)
node.setupParameter("time_window", 9999999999.0)
node.setupParameter("tf_base", '/comau_smart_six/base_link')
node.setupParameter("tf_target", '/comau_smart_six/link6')
node.setupParameter("default_image_format", 'jpg')
node.setupParameter("default_image_format_depth", 'png')
default_image_size = node.setupParameter("default_image_size", '640;480', array_type=int)
node.setupParameter("default_image_padding", 5)

print("Dataset: {}".format(timestamp))

tf_base = node.getParameter("tf_base")
tf_list = [node.getParameter("tf_target")]


topic_map = {
    # '/simple_message_stream/comau_smart_six_inner_message': eval("String"),
    # '/comau_smart_six/joint_states': eval("JointState"),
    # '/atift': eval("Twist"),
    # '/tactile': eval("Float64MultiArray"),
    # '/insertion_control/estimated_twist': eval("Twist"),
    # '/insertion_control/regulation_error': eval("Twist"),
    '/usb_cam_2/image_raw/compressed': eval('CompressedImage')
    # 'camera/rgb/image_raw/compressed': eval('CompressedImage')
    #'/darknet_rgb_detector/predictions': eval('Float64MultiArray')
    #'/camera/depth/image_raw': eval('Image')
}


container_map = {}
callback_map = {}
callback_managers = {}
data_map = {}
datatf_map = {}
times = []


def pushNewData():
    for topic, data in container_map.iteritems():
        print(topic, "DARTA")
        data_map[topic].append(data['value'])

        print(topic, len(data_map[topic]))
    for tf in tf_list:
        datatf_map[tf].append(tf_frames[tf])
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
        if data_type == PyKDL.Frame:
            return transformations.KDLtoNumpyVector(msg).reshape(7)
        if data_type == Twist:
            if msg is None:
                return np.zeros(6)
            else:
                return np.array([
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z
                ]).T
        if data_type == JointState:
            arr = np.array([])
            if msg:
                if len(msg.position) > 0:
                    arr = np.asarray(msg.position)
                if len(msg.velocity) > 0:
                    arr = np.hstack((arr, np.asarray(msg.velocity)))
            return arr.T

        if data_type == Float64MultiArray:
            if msg is None:
                return np.zeros(16)
            else:
                return np.array([msg.data]).T

        if data_type == CompressedImage:
            if msg is None:
                return np.zeros((default_image_size[1], default_image_size[0]))
            else:
                np_arr = np.fromstring(msg.data, np.uint8)
            try:
                return cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            except:
                return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if data_type == Image:
            if msg is None:
                return np.zeros((default_image_size[1], default_image_size[0]))
            try:
                return CV_BRIDGE.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                try:
                    return CV_BRIDGE.imgmsg_to_cv2(msg, "16UC1")
                except:
                    print("ERROR NO CONVERSION FOR IMAGE AVAILABLE!")
        if data_type == Float64MultiArray:
            if msg is None:
                return np.zeros(1)
            try:
                return msg.data.ravel()
            except:
                print("ERROR CONVERTIN Float64MultiArray ")

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
                if 'depth' in topic:
                    image_format = node.getParameter(
                        "default_image_format_depth"
                    )

                image_number_padding = node.getParameter(
                    "default_image_padding"
                )

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

for tf in tf_list:
    datatf_map[tf] = []

print("Started")
while node.isActive():
    if node.getElapsedTimeInSecs() >= node.getParameter("time_window"):
        print("Time Window")
        break
    try:
        found = True
        tf_frames = {}
        for tf in tf_list:
            tf_frame = node.retrieveTransform(
                tf, tf_base, -1)
            if tf_frame != None:
                tf_frames[tf] = tf_frame
            else:
                found = False
                break

        if found:
            pushNewData()
            print("Sync time ready!")

        node.tick()
    except KeyboardInterrupt:
        print("Interrupt")

print("Ended")

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

for tf, data in datatf_map.iteritems():
    for i in range(0, len(datatf_map[tf])):
        time = times[i]
        data_manager.pushData(
            'tf#' + tf,
            time,
            datatf_map[tf][i],
            PyKDL.Frame
        )

data_manager.setTimes(times)
data_manager.saveData()
