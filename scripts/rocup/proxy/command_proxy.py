#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import pkgutil
import rospy
from superros.logger import Logger
from rocup.msg import TextCommandAction, TextCommandGoal, TextCommandResult
import actionlib
from rocup.proxy.proxy_message import SimpleMessage
from actionlib_msgs.msg import GoalStatus
import random
import uuid
import json


def goalStatusString(state):
    for member in GoalStatus.__dict__.iteritems():
        if state == getattr(GoalStatus, member[0]):
            return member[0]


class CommandProxyConfiguration:
    COMMAND_PROXY_BASE_NAME = "/command_proxy/"

    @staticmethod
    def getProxyName(namespace):
        return CommandProxyConfiguration.COMMAND_PROXY_BASE_NAME + namespace


class CommandMessage(object):
    COMMAND_STATUS_UNKNOWN = -1
    COMMAND_STATUS_OK = 0
    COMMAND_STATUS_ERROR = 1

    def __init__(self, string_message, id=None):
        self.string_message = string_message
        self.id = id
        self.status = CommandMessage.COMMAND_STATUS_UNKNOWN
        self.error_message = ""
        self.response_data = {}
        if self.id == None:
            self.id = uuid.uuid4().hex

    def getSentMessage(self):
        return SimpleMessage.messageFromString(self.string_message)

    def addResponseData(self, key, value):
        self.response_data[key] = value

    def setStatus(self, status):
        self.status = status

    def setErrorMessage(self, error_message):
        self.error_message = error_message

    def __str__(self):
        return "Message['{}', {} , {}, {}]".format(
            self.string_message,
            self.id,
            "OK" if self.status == CommandMessage.COMMAND_STATUS_OK else "ERROR",
            self.error_message
        )

    def toActionMessage(self):
        return TextCommandGoal(
            text_command=self.string_message,
            id=self.id
        )

    def toActionResult(self, error_message=""):
        if error_message != "":
            return TextCommandResult(
                text_command=self.string_message,
                id=self.id,
                status=TextCommandResult.STATUS_ERROR,
                error_message=error_message,
                result_data=json.dumps(self.response_data)
            )
        else:
            return TextCommandResult(
                text_command=self.string_message,
                id=self.id,
                status=TextCommandResult.STATUS_OK,
                result_data=json.dumps(self.response_data)
            )

    @staticmethod
    def buildFromActionMessage(action_msg):
        return CommandMessage(
            string_message=str(action_msg.text_command),
            id=str(action_msg.id)
        )

    @staticmethod
    def buildFromActionResult(result):
        command = CommandMessage(
            string_message=str(result.text_command),
            id=str(result.id)
        )
        command.response_data = json.loads(result.result_data)
        command.setStatus(CommandMessage.COMMAND_STATUS_OK)
        if result.status == TextCommandResult.STATUS_ERROR:
            command.setStatus(CommandMessage.COMMAND_STATUS_ERROR)
            command.setErrorMessage(result.error_message)
        return command


class CommandProxyClient:

    def __init__(self, namespace):
        self.namespace = namespace
        self.action_client = actionlib.SimpleActionClient(
            CommandProxyConfiguration.getProxyName(self.namespace),
            TextCommandAction
        )
        Logger.log(
            "Command Proxy '{}' is waiting for server...".format(namespace))
        self.action_client.wait_for_server()
        Logger.log("Command Proxy '{}' ready!".format(namespace))
        #####

        self.feedback_callbacks = []
        self.done_callbacks = []
        self.active_callbacks = []

    def registerDoneCallback(self, callback):
        self.done_callbacks.append(callback)

    def registerActiveCallback(self, callback):
        self.active_callbacks.append(callback)

    def registerFeedbackCallback(self, callback):
        self.feedback_callbacks.append(callback)

    def sendCommand(self, text_command="", id=None):
        command = CommandMessage(
            string_message=text_command,
            id=id
        )
        self.action_client.send_goal(command.toActionMessage(),
                                     done_cb=self._done_callback,
                                     active_cb=self._active_callback,
                                     feedback_cb=self._feedback_callback)
        return command

    def _feedback_callback(self, feedback):
        pass

    def _done_callback(self, status, result):
        for c in self.done_callbacks:
            c(CommandMessage.buildFromActionResult(result))

    def _active_callback(self):
        pass


class CommandProxyServer:
    COMMAND_PROXY_BASE_NAME = "/command_proxy/"

    def __init__(self, namespace):
        self.namespace = namespace
        self.action_server = actionlib.SimpleActionServer(
            CommandProxyConfiguration.getProxyName(self.namespace),
            TextCommandAction,
            auto_start=False
        )
        #####
        self.active_goal = None
        self.command_ready_callbacks = []

        ####
        self.action_server.register_goal_callback(self._goal_callback)
        self.action_server.register_preempt_callback(self._preempt_callback)
        self.action_server.start()

    def registerCallback(self, callback):
        self.command_ready_callbacks.append(callback)

    def clearCallbacks(self):
        self.command_ready_callbacks = []

    def _goal_callback(self):
        if not self.action_server.is_active():
            # Logger.log("New goal accepted!")
            self._setNewGoal(self.action_server.accept_new_goal())
        else:
            Logger.warning("New goal not accepted! Another goal is active")

    def _preempt_callback(self):
        if self.action_server.is_active():
            self.action_server.set_preempted()
            self._setNewGoal(self.action_server.accept_new_goal())

    def _setNewGoal(self, goal):
        self.active_goal = goal

        for c in self.command_ready_callbacks:
            c(CommandMessage.buildFromActionMessage(goal))

    def resolveCommand(self, command):
        if self.action_server.is_active():
            # print("### RESOLVE ### ", str(command))
            try:
                result = command.toActionResult()
                self.action_server.set_succeeded(result=result)
            except:
                self.rejectCommand(command, reject_message="NOT_ACCEPTED")

    def rejectCommand(self, command, reject_message="UNKONWN_ERROR"):
        # print("### REJECT ### ", str(command))
        try:
            result = command.toActionResult(reject_message)
            self.action_server.set_aborted(result=result)
        except:
            self.action_server.set_aborted()
