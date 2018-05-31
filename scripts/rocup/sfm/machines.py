import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import PyKDL
import tf
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import superros.transformations as transformations
import rocup.robots.controllers as controllers
import rocup.sfm.machinescommands as machinescommands
from superros.logger import Logger
from rocup.srv import SFMCommand, SFMCommandResponse
import transitions
import random
import json


class SFMachineState(object):
    def __init__(self, name):
        self.name = name
        self.out_edges = []
        self.in_edges = []
        self.edges = []

    def addOutEdge(self, edge):
        self.out_edges.append(edge)
        self.edges.append(edge)

    def addInEdge(self, edge):
        self.in_edges.append(edge)
        self.edges.append(edge)

    def isLeaf(self):
        return len(self.out_edges) <= 0

    def isRoot(self):
        return len(self.in_edges) <= 0

    def __str__(self):
        return "S#{}".format(self.name)

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)


class SFMachineEdge(object):

    def __init__(self, name, s1, s2):
        self.name = name
        self.state1 = s1
        self.state2 = s2
        self.state1.addOutEdge(self)
        self.state2.addInEdge(self)

    def isValid(self):
        return self.state1 != None and self.state2 != None

    def __str__(self):
        return "E#{}:{},{}".format(self.name, self.state1.name, self.state2.name)


class SFMachineGraph(object):

    def __init__(self):
        self.states = {}
        self.edges = []

    def addState(self, name):
        state = SFMachineState(name)
        self.states[name] = state
        return state

    def addEdge(self, name, state1, state2):
        if isinstance(state1, str):
            state1 = self.states.get(state1)
        if isinstance(state2, str):
            state2 = self.states.get(state2)
        edge = SFMachineEdge(name, state1, state2)
        self.edges.append(edge)
        return edge

    def dump(self):
        g = {
            "states": [],
            "edges": []
        }
        for s in self.states:
            state = self.states[s]
            item = {
                "name": state.name,
                "leaf": state.isLeaf(),
                "root": state.isRoot()
            }
            g["states"].append(item)

        for edge in self.edges:
            if edge.isValid():
                item = {
                    "name": edge.name,
                    "start": edge.state1.name,
                    "end": edge.state2.name
                }
                g["edges"].append(item)
        return g


class SFMachineRequest(object):

    def __init__(self):
        self.command = ""
        self.data = None


class SFMachineResponse(object):

    def __init__(self):
        self.status = 0
        self.data = None


class SFMachine(transitions.Machine):

    def __init__(self, name="SFM_{}".format(random.randint(0, 99999)), model=None):
        super(SFMachine, self).__init__(model=model)
        self.add_state("a")
        self.graph = SFMachineGraph()
        # print(" \n******************** {} ********************\n".format(name))
        self.service = rospy.Service(
            name, SFMCommand, self._command_cb)

    def _command_cb(self, msg):
        response = self.parseRequest(msg.request)
        return SFMCommandResponse(json.dumps(response.data))

    def parseRequest(self, request_str):
        response = SFMachineResponse()
        request = None
        try:
            request = json.loads(request_str,)
        except:
            request = None
        if request:
            response.data = self.consumeCommand(
                request["command"], request["data"])
        return response

    def consumeCommand(self, command_name, data):
        command = machinescommands.SFMachineCommand(command_name, data)
        return command.execute(self)

    def create(self):
        dump = self.graph.dump()
        for s in dump.get("states"):
            self.add_state(s["name"])

        for e in dump.get("edges"):
            self.add_transition(e["name"], e["start"], e["end"])

    def addState(self, state):
        return self.graph.addState(state)

    def addTransition(self, trigger, source, dest):
        return self.graph.addEdge(trigger, source, dest)

    def getModel(self):
        return self.models[0]

    def loop(self):
        if self.getModel():
            method = None
            try:
                method = getattr(self.getModel(), "on_loop_" +
                                 self.getModel().state)
            except (AttributeError) as e:
                pass
            if method != None:
                method()
