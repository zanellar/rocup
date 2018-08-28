#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy

"""Example: 
NODE:       node_rate = Parameters.get(obj=robot_name, param="NODE_FREQUENCY")
        or 
            world_id = Parametrs.get("WORLD_ID")
        or
            Parameters.change(obj=robot_name, 
                                param="NODE_FREQUENCY", 
                                value=hz) 
        or
            Parameters.change(param="WORLD_ID", 
                              value=tf_name) 
                                
LAUNCH:     <param name="$(arg robot_name)/TRAJECTORY_TIME" value="5"/>
"""


class Parameters:

    elements = {

        #⬢⬢⬢⬢⬢➤ Robots
        "COMAU_NAME":                   "comau_smart_six",
        "GRASSHOPPER_NAME":             "grasshopper",
        "SCHUNK_NAME":                  "schunk_pg70",
        "BONMET_NAME":                  "bonmetc60",

        #⬢⬢⬢⬢⬢➤ Sensors
        # ... add here the sensors names
        "WRIST_FT_SENSOR":              "atift",
        "TACTILE":                      "tactile"

        #⬢⬢⬢⬢⬢➤ Subtasks
        # ... add here the subtasks names
    }

    default_params = {


        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ ROBOTS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

        #⬢⬢⬢⬢⬢➤ COMAU
        elements["COMAU_NAME"] + "/TRAJECTORY_TIME":    5,
        elements["COMAU_NAME"] + "/NODE_FREQUENCY":     300,
        elements["BONMET_NAME"] + "/TRAJECTORY_POINTS":    None,       # []
        elements["COMAU_NAME"] + "/TOOLS":              {
            "none": [0, 0, 0, 0, 0, 0, 1],
            "gripper": [0, 0, -0.2995, 0, 0, 0, 1],
            "camera": [-0.000825, -0.129085, -0.168954, 0.018076, 1.000534, 0.007385, 0.003956],
            "dynamic": [0, 0, 0, 0, 0, 0, 1],
        },

        #⬢⬢⬢⬢⬢➤ GRASSHOPPER
        elements["GRASSHOPPER_NAME"] + "/TRAJECTORY_TIME":  20,
        elements["GRASSHOPPER_NAME"] + "/NODE_FREQUENCY":   100,
        elements["BONMET_NAME"] + "/TRAJECTORY_POINTS":    1,       # []
        elements["GRASSHOPPER_NAME"] + "/TOOLS":              {
            "none": [0, 0, 0, 0, 0, 0, 1],
            "gripper": [0, 0, 0, 0, 0, 0, 1],
            "camera": [0, 0, 0, 0, 0, 0, 1],
            "dynamic": [0, 0, 0, 0, 0, 0, 1],
        },


        #⬢⬢⬢⬢⬢➤ SCHUNK
        elements["SCHUNK_NAME"] + "/TRAJECTORY_TIME":       1,
        elements["SCHUNK_NAME"] + "/NODE_FREQUENCY":        2,
        elements["BONMET_NAME"] + "/TRAJECTORY_POINTS":    1,       # []
        elements["SCHUNK_NAME"] + "/TOOLS":              {
            "none": [0, 0, 0, 0, 0, 0, 1],
            "gripper": [0, 0, 0, 0, 0, 0, 1],
            "camera": [0, 0, 0, 0, 0, 0, 1],
            "dynamic": [0, 0, 0, 0, 0, 0, 1],
        },


        #⬢⬢⬢⬢⬢➤ BONMET
        elements["BONMET_NAME"] + "/NODE_FREQUENCY":       300,     # [hz]
        elements["BONMET_NAME"] + "/TRAJECTORY_TIME":      5,       # [sec]
        elements["BONMET_NAME"] + "/TRAJECTORY_POINTS":    1,       # []
        elements["BONMET_NAME"] + "/TOOLS": {
            "none": [0, 0, 0, 0, 0, 0, 1],
            "gripper": [0, -0.068, 0.37, 0, 0, 1, 0],
            "camera": [-0.002746, -0.117399, 0.297752, 0.040049, 0.016541, 0.019843, 1.01652],
            "dynamic": [0, 0, 0, 0, 0, 0, 1],
        },


        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ SENSORS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ... add here the sensors parametrs by name
        "WRIST_FT_SENSOR_POSITION_WRT_EEF":  [0, 0, 0.107],  # position (x,y,z) with respect to the end-effector

        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ SUBTASKS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ... add here the subtasks parametrs by name

        # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ OTHERS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
        # ... add here the system parametrs
        "WORLD_FRAME_ID":               "world"
    }

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    @staticmethod
    def get(param, obj=""):
        """ Ritorna il valore del parametro richiesto.  """

        try:
            if param in Parameters.elements.keys():    # a name is requested
                return rospy.get_param(param, Parameters.elements[param])
            elif obj in Parameters.elements.values():   # a elements's parameter is requested
                param = "{}/{}".format(obj, param)
            return rospy.get_param(param, Parameters.default_params[param])
        except Exception as e:
            print(e)
            return None

    @staticmethod
    def change(param, value, obj=""):
        """ Ritorna il valore del parametro richiesto. """
        try:
            if obj in Parameters.elements.values():   # a elements's parameter is requested
                param = "{}/{}".format(obj, param)
            rospy.set_param(param, value)
            return True
        except Exception as e:
            print(e)
            return False
