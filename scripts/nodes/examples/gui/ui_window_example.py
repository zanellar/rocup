#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
import sys
from rocup.ui.pyqt import PyQtWindow
from superros.comm import RosNode

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("ui_window_example")

ui_file = node.getFileInPackage("rocup", "data/ui/main.ui")
w = PyQtWindow(uifile=ui_file)


w.run()
