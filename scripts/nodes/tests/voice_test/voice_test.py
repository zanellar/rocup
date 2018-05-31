#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from superros.logger import Logger
import superros.transformations as transformations
from rocup.utils.devices import ForceSensor, Joystick
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import PyKDL
import copy
import math
import tf

import speech_recognition as sr
import pyttsx
import threading
import os


class VoiceAssistant(threading.Thread):
    def __init__(self):
        super(VoiceAssistant, self).__init__()
        self.engine = pyttsx.init()
        self.say_list = []
        self.daemon = True
        # voices = self.engine.getProperty('voices')
        # for voice in voices:
        #     print voice.name
        #     if voice.name == 'italian':
        #         print("Setting italian")
        self.engine.setProperty('voice', 'mb-it3')
        self.engine.setProperty('rate', 120)

    def add_say(self, msg):
        self.engine.say(msg)
        self.engine.say("   ")
        self.engine.runAndWait()
        time.sleep(1)
        self.engine.stop()

    def _callback(self, msg):
        self.add_say(msg.data)

    def run(self):
        while True:
            if len(self.say_list) > 0:
                print("Saiynh", self.say_list[0])
                self.engine.say(self.say_list[0])
                self.engine.runAndWait()
                time.sleep(1)
                self.say_list.remove(self.say_list[0])

#############################################################################
#############################################################################
#############################################################################
#############################################################################


def listenCallback(msg):
    os.system("espeak -v it-It '{}'".format(msg.data))


rospy.init_node('voice_test', anonymous=True)
rate = rospy.Rate(30)  # 10hz

say_pub = rospy.Publisher('/jarvis/message',
                          String, queue_size=1)


# Voice Assistant
assistant = VoiceAssistant()
listen_sub = rospy.Subscriber(
    '/jarvis/listen', String, listenCallback, queue_size=1)

# assistant.add_say("li mortacci")

# Speech Recognition
r = sr.Recognizer()
source = sr.Microphone()

# Loop
start_time = rospy.get_time()
while not rospy.is_shutdown():

    try:
        with sr.Microphone() as source:
            print("Say something!")
            audio = r.listen(source, timeout=0.2)

    except sr.WaitTimeoutError as e:
        continue

    try:
        recognized = r.recognize_google(audio, language="it-IT")
        say_msg = String()
        say_msg.data = recognized
        say_pub.publish(say_msg)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print(
            "Could not request results from Google Speech Recognition service; {0}".format(e))
    rate.sleep()
