import rospy
import numpy as np
from superros.logger import Logger
from rocup.srv import SFMCommand, SFMCommandResponse
import transitions
import json


class SFMachineCommand(object):
    def __init__(self, name="", data=None):
        self.name = name
        self.data = data

    def execute(self, sfm_machine):
        if self.name == "getGraph":
            return sfm_machine.graph.dump()
        if self.name == "getCurrentState":
            return {"name": sfm_machine.getModel().state}
        return None
