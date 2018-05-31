#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
import sys
from rocup.msg import UiEvent
from rocup.ui.uiproxy import UiProxy
import signal

#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################


class PyQtNamesMap(object):
    class_map = {
        "QPushButton": "button"
    }

    @staticmethod
    def getClassName(qt_name):
        if qt_name in PyQtNamesMap.class_map:
            return PyQtNamesMap.class_map[qt_name]
        return qt_name

#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################


class PyQtWindow(QtGui.QMainWindow):
    def __init__(self, uifile, namespace="default"):
        self.qt_application = QApplication(sys.argv)
        super(PyQtWindow, self).__init__()
        uic.loadUi(uifile, self)

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.ui_proxy = UiProxy(namespace)
        self.widget_map = {}
        self.ui_proxy.registerUiCallback(self._dataCallback)
        self.visitWidgets()

        # try:
        #     import qdarkstyle
        #     if qdarkstyle:
        #         self.qt_application.setStyleSheet(
        #             qdarkstyle.load_stylesheet(pyside=False)
        #         )
        # except:
        #     pass

    def _dataCallback(self, evt):
        self.updateUI(evt.name, evt.value, evt.value_string)

    def updateUI(self, widget_name, value, value_string):
        if widget_name in self.widget_map:
            w = self.widget_map[widget_name]
            if isinstance(w, QDial):
                w.setValue(float(value))
            if isinstance(w, QProgressBar):
                w.setValue(float(value))
            if isinstance(w, QLabel):
                w.setText(str(value) + str(value_string))

    def _getSenderSimpleName(self):
        return PyQtNamesMap.getClassName(
            self.sender().metaObject().className())

    def _clickedCallback(self):
        self.ui_proxy.sendEventToApplication(
            sender_name=self.sender().objectName(),
            sender_type=self._getSenderSimpleName(),
            sender_event="click"
        )

    def _changeCallback(self, value):
        w = self.sender()
        if isinstance(w, QSlider) or isinstance(w, QDial):
            value = value
        if isinstance(w, QCheckBox):
            value = w.isChecked()

        self.ui_proxy.sendEventToApplication(
            sender_name=self.sender().objectName(),
            sender_type=self._getSenderSimpleName(),
            sender_event="change",
            value=value
        )

    def _textChangeCallback(self):
        w = self.sender()
        if isinstance(w, QTextEdit):
            text = w.toPlainText()
        if isinstance(w, QLineEdit):
            text = w.text()
        if isinstance(w, QComboBox):
            text = w.currentText()

        self.ui_proxy.sendEventToApplication(
            sender_name=self.sender().objectName(),
            sender_type=self._getSenderSimpleName(),
            sender_event="change",
            value=text
        )

    def initButtons(self, buttons_list):
        for button in buttons_list:
            button_handle = getattr(self, button)
            button_handle.clicked.connect(self._clickedCallback)

    def run(self):
        self.show()
        sys.exit(self.qt_application.exec_())

    def _manageButtonEvents(self, button):
        button.clicked.connect(self._clickedCallback)

    def _manageSliderEvents(self, slider):
        slider.valueChanged.connect(self._changeCallback)

    def _manageTextEvents(self, slider):
        slider.textChanged.connect(self._textChangeCallback)

    def _manageStateEvents(self, slider):
        slider.stateChanged.connect(self._changeCallback)

    def _manageComoboEvents(self, combo):
        combo.currentIndexChanged.connect(self._textChangeCallback)

    def _manageGenericWidgetEvents(self, widget):
        if widget.metaObject().className() == "QPushButton":
            self._manageButtonEvents(widget)
        if widget.metaObject().className() == "QToolButton":
            self._manageButtonEvents(widget)
        if widget.metaObject().className() == "QCommandLinkButton":
            self._manageButtonEvents(widget)
        if widget.metaObject().className() == "QSlider":
            self._manageSliderEvents(widget)
        if widget.metaObject().className() == "QDial":
            self._manageSliderEvents(widget)
        if widget.metaObject().className() == "QTextEdit":
            self._manageTextEvents(widget)
        if widget.metaObject().className() == "QLineEdit":
            self._manageTextEvents(widget)
        if widget.metaObject().className() == "QCheckBox":
            self._manageStateEvents(widget)
        if widget.metaObject().className() == "QSpinBox":
            self._manageSliderEvents(widget)
        if widget.metaObject().className() == "QDoubleSpinBox":
            self._manageSliderEvents(widget)
        if widget.metaObject().className() == "QComboBox":
            self._manageComoboEvents(widget)

    def visitWidgets(self, root=None):
        if root == None:
            root = self
        for w in root.children():
            name = str(w.objectName())
            print(name, w)
            self.widget_map[name] = w
            self._manageGenericWidgetEvents(w)
            self.visitWidgets(w)
