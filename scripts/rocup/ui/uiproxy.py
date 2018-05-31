#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
import sys
from rocup.msg import UiEvent
from PyQt4.QtCore import QString
import rospy


class UiProxy(object):

    def __init__(self, namespace="default"):
        self.a = 0
        self.name = "/ui/" + namespace + "/"
        self.ui_publisher = rospy.Publisher(
            self.name, UiEvent, queue_size=1000)
        self.ui_subscriber = rospy.Subscriber(
            self.name, UiEvent, self._uiCallback)
        self.callbacks_for_ui = []
        self.callbacks_for_application = []

    def _uiCallback(self, msg):
        if msg.direction == UiEvent.DIRECTION_UI_TO_APPLICATION:
            for c in self.callbacks_for_application:
                c(msg)
        if msg.direction == UiEvent.DIRECTION_APPLICATION_TO_UI:
            for c in self.callbacks_for_ui:
                c(msg)

    def registerUiCallback(self, ui_callback):
        self.callbacks_for_ui.append(ui_callback)

    def registerApplicationCallback(self, application_callback):
        self.callbacks_for_application.append(application_callback)

    @staticmethod
    def CreateEvent(sender_name="", sender_type="", sender_event="", value=None):
        event = UiEvent()
        event.name = str(sender_name)
        event.type = str(sender_type)
        event.event = str(sender_event)
        if value:
            if isinstance(value, str) or isinstance(value, QString):
                event.value_string = str(value)
            else:
                try:
                    event.value = float(value)
                except:
                    pass
        return event

    def sendEventToApplication(self, sender_name="", sender_type="", sender_event="", value=None):
        event = UiProxy.CreateEvent(
            sender_name=sender_name,
            sender_type=sender_type,
            sender_event=sender_event,
            value=value
        )
        event.direction = UiEvent.DIRECTION_UI_TO_APPLICATION
        self.ui_publisher.publish(event)

    def sendDataToUi(self, field_name="", value=None):
        event = UiProxy.CreateEvent(
            sender_name=field_name,
            sender_type="update",
            value=value
        )
        event.direction = UiEvent.DIRECTION_APPLICATION_TO_UI
        self.ui_publisher.publish(event)
