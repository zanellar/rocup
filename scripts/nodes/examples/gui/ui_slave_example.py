#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
import sys
from rocup.ui.uiproxy import UiProxy
from superros.comm import RosNode
import math

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("ui_slave_example")
node.setHz(2)

flag1 = True

# Receive Data From Ui


def eventCallback(evt):
    global flag1
    print("*** New Event Received ***")
    print(evt)

    if evt.name == "flag1":
        flag1 = evt.value > 0


ui_proxy = UiProxy()

ui_proxy.registerApplicationCallback(eventCallback)

while node.isActive():
    s = 100 * math.sin(node.getElapsedTimeInSecs() * 1)

    if flag1:
        # Send data to UI
        ui_proxy.sendDataToUi("label", s)

    node.tick()
