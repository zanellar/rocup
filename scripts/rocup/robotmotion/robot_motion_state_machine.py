#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import pkgutil
import rospy
import numpy
from numpy import *
from numpy.linalg import *
from scipy.interpolate import interp1d
import math
import pprint
import time
import threading
import tf
pp = pprint.PrettyPrinter(indent=4)
import PyKDL
from PyKDL import Frame, Vector, Rotation

from transitions import Machine

from rocup.robots.controllers import RobotController, GeneralIKService, Robot
from rocup.robots.trajectories import FuzzyTrajectoryGenerator
from rocup.robots.market import RobotMarket
import superros.transformations as transformations
from superros.logger import Logger
import rocup.sfm.machines as machines
from rocup.param.global_parameters import Parameters
from rocup.robotmotion.command_interpreter import CommandInterpreter
from rocup.proxy.alarm_proxy import AlarmProxy

from rocup.srv import IKService, IKServiceResponse
from std_srvs.srv import Trigger

from std_msgs.msg import Header
from std_msgs.msg import String, Bool
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from rocup.msg import *
from rocup.msg import RobotFollow
from rocup.msg import RobotMotionResult, RobotMotionFeedback


WORLD_FRAME_ID = Parameters.get("WORLD_FRAME_ID")


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    STATE MACHINE DEFINITION    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class SFMachineRobotDefinition(object):

    # TOOL = PyKDL.Frame()
    # STANDARD_ELEVATION = PyKDL.Frame()
    # STANDARD_ELEVATION.p.z(0.6)

    def __init__(self, robot):
        # tf Listener
        self.listener = tf.TransformListener()

        # robot
        self.robot = robot
        self.robot_name = robot.getName()

        # trajectory
        self.trajectory_generator = FuzzyTrajectoryGenerator(self.robot)
        self.trajectory = []
        self.generate_trajectory_failure = False
        self.trajectory_correct = False

        # command
        self.current_command = None
        self.interpreter = CommandInterpreter(robot_name=self.robot_name,
                                              robot_object=self.robot)

        # frames
        self.ee_frame = None

        # target
        self.target = None
        self.target_q = None
        self.dist_target = -1

        # parameters
        self.node_frequency = Parameters.get(obj=self.robot_name, param="NODE_FREQUENCY")
        self.trajectory_points = Parameters.get(obj=self.robot_name, param="TRAJECTORY_POINTS")
        self.trajectory_time = Parameters.get(obj=self.robot_name, param="TRAJECTORY_TIME")
        self.joints_tollerance = self.robot.getParam("joints_tollerance")

        # timer
        self.init_generate_timer = 15 * self.node_frequency  # [sec]
        self.init_start_moving_timer = 10 * self.node_frequency  # [sec]

        # external callback
        self.feedback_ext_callback = None
        self.feedback = RobotMotionFeedback()
        self.result_ext_callback = None
        self.result = RobotMotionResult()

        # error
        self.err = self.result.NO_ERROR

        # noise velocity in steady state
        self.vel_noise_mean = 0.015

        # alarm
        self.in_allarm = False
        self.alarm_proxy = AlarmProxy(self.robot_name)
        self.alarm_proxy.registerAlarmCallback(self.alarm_callback)
        self.alarm_proxy.registerResetCallback(self.alarm_reset_callback)

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PUBLIC METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def uploadCurrentCommand(self, command):
        if self.current_command != None:
            self.error("COMMAND_CONFLICT")
        else:
            self.current_command = command

    def isTrajectoryAdmissible(self):
        if self.trajectory_generator != None:
            if self.trajectory_generator.isTrajectoryAvailable():
                return self.trajectory_correct
        else:
            return False

    def applyTrajectory(self):
        if len(self.trajectory_generator.current_trajectory) > 0:
            self.trajectory_generator.applyTrajectory(rospy.Time.now())

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ PRIVATE METHODS ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    def _sendFeedback(self, text="", state="", command="", status=0, progress=-1.0, warning=0):
        if self.feedback_ext_callback:
            self.feedback.current_state = state
            self.feedback.text = text
            self.feedback.executing_command = command
            self.feedback.status = status
            self.feedback.progress = progress
            self.feedback.warning = warning
            self.feedback_ext_callback(self.feedback)
        else:
            Logger.error("External callback for FEEDBACK update NOT found!")

    def _sendResult(self, success=False, command="", error=0):
        if self.current_command != None:
            if self.result_ext_callback:
                q = self.robot.getController().getQ()
                pose = geometry_msgs.msg.Pose()
                eef_frame = self.robot.getEEFrame()
                pose.position.x = eef_frame.p.x()
                pose.position.y = eef_frame.p.y()
                pose.position.z = eef_frame.p.z()
                eef_quaternion = eef_frame.M.GetQuaternion()
                pose.orientation.x = eef_quaternion[0]
                pose.orientation.y = eef_quaternion[1]
                pose.orientation.z = eef_quaternion[2]
                pose.orientation.w = eef_quaternion[3]

                self.result.pose = pose
                self.result.q.data = list(q)
                self.result.success = success
                self.result.error = error
                self.result.executed_command = command
                self.result.q_target_distance = self.dist_target

                self.result_ext_callback(self.result)

                succ = "\033[91m" + "\033[92m" * success + "\033[1m\033[4m{}\033[0m".format(success)
                print("{}: {}".format(self.current_command, succ))
            else:
                Logger.error("External callback for RESULT update NOT found!")
        self.current_command = None

    def _computeProgress(self):
        current_q = self.robot.getController().getQ()
        target_q = self.target_q
        initial_q = self.initial_q
        self.movement_progress = 100.0 * (1.0 - abs((norm(subtract(current_q, target_q))) /
                                                    (norm(subtract(initial_q, target_q)))))

    def _isOnTarget(self, tollerance=None):
        if tollerance == None:
            tollerance = self.joints_tollerance
        current_q = self.robot.getController().getQ()
        self.dist_target = norm(subtract(self.target_q, current_q))
        sys.stdout.write("Working --> Distance From Target: {} \r".format(self.dist_target))
        sys.stdout.flush()
        return self.dist_target < tollerance

    def _generateTrajectoryFromCommand(self):
        self.initial_q = self.robot.getController().getQ()
        self.trajectory_generator.clearTrajectory()
        self.generate_trajectory_failure = False
        steps = self.trajectory_time * self.node_frequency
        if self.target:
            self.trajectory_points = Parameters.get(obj=self.robot_name, param="TRAJECTORY_POINTS")
            if self.trajectory_points is None:
                self.trajectory_points = self.node_frequency * self.trajectory_time
            try:
                if self.interpreter.trajectory_specs == "linear_joints":
                    self.trajectory = self.trajectory_generator.generateTrajectoryFromShapes(self.initial_q,
                                                                                             self.target,
                                                                                             self.trajectory_points,
                                                                                             1)
                elif self.interpreter.trajectory_specs == "linear_frames_to_free_joints":
                    self.target = self.target * self.robot.getCurrentTool().Inverse()
                    self.trajectory = self.trajectory_generator.generateTrajectoryFromFrames(self.initial_q,
                                                                                             self.robot.getEEFrame(),
                                                                                             self.target,
                                                                                             self.node_frequency,
                                                                                             self.trajectory_time)
                elif self.interpreter.trajectory_specs == "free_frames_to_linear_joints":
                    self.target = self.target * self.robot.getCurrentTool().Inverse()
                    self.trajectory = self.trajectory_generator.generateTrajectoryFreeFromFrames(self.initial_q,
                                                                                                 self.robot.getEEFrame(),
                                                                                                 self.target,
                                                                                                 steps=steps,
                                                                                                 agumented_middle_joints=True,
                                                                                                 middle_frames=10,
                                                                                                 number_of_tries=1,
                                                                                                 perturbate_middle_frames=False)
                elif self.interpreter.trajectory_specs == "target_frame_to_target_joints":
                    self.target = self.target * self.robot.getCurrentTool().Inverse()
                    self.trajectory = self.trajectory_generator.generateTrajectoryFreeFromFrames(self.initial_q,
                                                                                                 self.robot.getEEFrame(),
                                                                                                 self.target,
                                                                                                 steps=1,
                                                                                                 agumented_middle_joints=False,
                                                                                                 middle_frames=20,
                                                                                                 number_of_tries=5,
                                                                                                 perturbate_middle_frames=False)
                else:
                    self.trajectory = []
            except Exception as e:
                Logger.error(e)
                self.trajectory = []
        else:
            self.trajectory = []

        # Trajectory target update
        if len(self.trajectory) > 0:
            self.target_q = self.trajectory_generator.getTargetQ()
        else:
            self.generate_trajectory_failure = True
            self.target_q = self.initial_q

    def alarm_callback(self, alarm_info):
        if alarm_info != self.alarm_proxy.NONE_ALARM:
            self.error("ROBOT_ALARM")

    def alarm_reset_callback(self):
        if self.state == "error" and self.in_allarm:
            Logger.warning("\nAlarm Reset\n")
            self.in_allarm = False
            # self.idle()

    # ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇ STATES ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ IDLE ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    # ➤ ➤ ➤ ➤ ➤ IDLE: enter
    def on_enter_idle(self):
        Logger.log("State:  IDLE")
        self._sendFeedback(state="idle")

    def on_loop_idle(self):
        pass

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ PREPARING ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ PREPARING: enter
    def on_enter_preparing(self):
        Logger.log("State:  PREPARING")
        self._sendFeedback(state="preparing")
        self.ee_frame = self.robot.getEEFrame()
        self.vel_noise_samples = []

    #➤ ➤ ➤ ➤ ➤ PREPARING: loop
    def on_loop_preparing(self):
        if self.vel_noise_samples >= 30:
            self.vel_noise_mean = mean(self.vel_noise_samples)
            self.idle()
        else:
            self.vel_noise_samples.append(self.robot.getController().getQ())

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ SENDCOMMAND ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ SENDCOMMAND: enter
    def on_enter_sendcommand(self):
        Logger.log("State:  SENDCOMMAND")
        self._sendFeedback(state="sendcommand")

    #➤ ➤ ➤ ➤ ➤ SENDCOMMAND: loop
    def on_loop_sendcommand(self):
        self._sendFeedback(status=self.feedback.WAITING_FOR_COMMAND)
        if self.current_command != None:
            self.checkcommand()
    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ CHECKCOMMAND ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ CHECKCOMMAND: enter
    def on_enter_checkcommand(self):
        Logger.log("State:  CHECKCOMMAND")
        self._sendFeedback(state="checkcommand",
                           command=self.current_command)
        self.target = self.interpreter.interpretCommand(self.current_command)

    #➤ ➤ ➤ ➤ ➤ CHECKCOMMAND: loop
    def on_loop_checkcommand(self):
        if self.interpreter.isCommandValid():
            self.generatetrajectory()
        else:
            self.error("NO_COMMAND")

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ GENERATETRAJECTORY ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ GENERATETRAJECTORY: enter
    def on_enter_generatetrajectory(self):
        Logger.log("State:  GENERATETRAJECTORY")
        self._sendFeedback(state="generatetrajectory",
                           command=self.current_command)

        self.generate_timer = self.init_generate_timer
        self.t_gen = -1
        self.trajectoryThread = threading.Thread(target=self._generateTrajectoryFromCommand)
        self.trajectoryThread.start()

    #➤ ➤ ➤ ➤ ➤ GENERATETRAJECTORY: loop
    def on_loop_generatetrajectory(self):
        self._sendFeedback(text=str(self.init_generate_timer),
                           state="generatetrajectory",
                           command=self.current_command,
                           status=self.feedback.WAITING_FOR_MOVING)

        t_new = self.generate_timer / self.node_frequency
        if self.t_gen != t_new:
            self.t_gen = t_new
            Logger.log("waiting start (time: {}s)".format(self.t_gen))
        if self.trajectory_generator.isTrajectoryAvailable():
            self.trajectoryThread.join()
            self.checktrajectory()
        elif self.generate_timer <= 0 or self.generate_trajectory_failure:
            self.trajectoryThread.join()
            self.error("NO_TRAJECTORY")

        self.generate_timer -= 1

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ CHECKTRAJECTORY ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ CHECKTRAJECTORY: enter
    def on_enter_checktrajectory(self):
        Logger.log("State:  CHECKTRAJECTORY")
        self._sendFeedback(state="checktrajectory")
        # print("{}".format(self.trajectory_generator.current_trajectory))

        if not self.trajectory_generator.computationSuccess():
            self.error("NO_TRAJECTORY")
        else:
            if self._isOnTarget():
                Logger.log("Already on Target")
                self._sendFeedback(state="checktrajectory",
                                   text="Already on Target")
                self.steady()
            else:
                self.trajectory_correct = True
                self.ready()

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ READY ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ READY: enter
    def on_enter_ready(self):
        Logger.log("State:  READY")
        self._sendFeedback(state="ready")

        self.start_moving_timer = self.init_start_moving_timer

        # if self.robot.getController().isMoving(epsilon=self.vel_noise_mean * 1.05): # TODO start-moving timer disabled!!!!!!!!!!!!!
        if len(self.trajectory_generator.current_trajectory) > 0:
            self.moving()

    #➤ ➤ ➤ ➤ ➤ READY: loop
    def on_loop_ready(self):
        self._sendFeedback(text=str(self.start_moving_timer),
                           state="ready",
                           status=self.feedback.WAITING_FOR_MOVING)

        # if self.robot.getController().isMoving(epsilon=self.vel_noise_mean * 1.05):
        if len(self.trajectory_generator.current_trajectory) > 0:
            self.moving()
        elif self.start_moving_timer <= 0:
            self.error("START_MOVING")

        self.start_moving_timer -= 1

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ MOVING ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ MOVING: enter
    def on_enter_moving(self):
        Logger.log("State:  MOVING")
        self._sendFeedback(state="moving", status=self.feedback.MOVING)
        delay = int(0.6 * float(self.trajectory_time * self.node_frequency))
        self.end_moving_timer = self.trajectory_time * self.node_frequency + delay
        self.movement_progress = 0
        # self.progressThread = threading.Thread(target=self._computeProgress)
        # self.progressThread.start()

    #➤ ➤ ➤ ➤ ➤ MOVING: loop
    def on_loop_moving(self):

        # Logger.log("waiting end (timer: {})".format(self.end_moving_timer))

        self._computeProgress()

        if self.end_moving_timer > 0:
            wrn = self.feedback.NO_WARNING
        else:
            wrn = self.feedback.WARNING_TRAJACTORY_TIME_OUT

        self._sendFeedback(text=str(self.end_moving_timer),
                           progress=self.movement_progress,
                           state="moving",
                           status=self.feedback.MOVING,
                           warning=wrn)

        if not self.robot.getController().isMoving(epsilon=self.vel_noise_mean * 1.05):
            if self._isOnTarget():
                print("On Target --> Distance: {}".format(self.dist_target))
                self.steady()
            elif self.end_moving_timer <= 0:
                print("Distance From Target: {}".format(self.dist_target))
                self.error("END_TRAJECTORY")
            else:
                self.end_moving_timer -= 1
        else:
            self.end_moving_timer -= 1

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ STEADY ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ STEADY: enter
    def on_enter_steady(self):
        self._sendFeedback(state="steady", status=self.feedback.STEADY)
        Logger.log("State:  STEADY")
        self._sendResult(success=True, command=self.current_command)
        self.trajectory_correct = False
        self.current_command = None
        self.idle()

    # #➤ ➤ ➤ ➤ ➤ STEADY: loop
    # def on_loop_steady(self):

    #
    #⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢ ERROR ⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢⬢
    #

    #➤ ➤ ➤ ➤ ➤ ERROR: enter
    def on_enter_error(self, error_type=""):
        Logger.log("State:  ERROR")
        self._sendFeedback(state="error")
        self.trajectory_generator.clearTrajectory()
        if error_type == "NO_COMMAND":
            self.err = self.result.ERROR_NO_COMMAND
            Logger.warning(
                "Command string NOT recognized. Waiting for a new command")
        elif error_type == "NO_TRAJECTORY":
            self.err = self.result.ERROR_NO_TRAJECTORY
            Logger.warning(
                "Trajectory NOT computed. Waiting for a new command")
        elif error_type == "START_MOVING":
            self.err = self.result.ERROR_START_MOVING
            Logger.warning(
                "Moving Time Out. Target NOT Reached.  Waiting for a new command")
        elif error_type == "END_TRAJECTORY":
            self.err = self.result.ERROR_END_TRAJECTORY
            Logger.error(
                "Trajectory Time Out. Target NOT Reached. Waiting for a new command")
        elif error_type == "ROBOT_ALARM":
            self.err = self.result.ERROR_ROBOT_ALARM
            self.in_allarm = True
            Logger.error(
                "\n\n\n ROBOT ALARM!!! \n Restore the machine and Reset the Alarm \n\n\n")
        elif error_type == "COMMAND_CONFLICT":
            self.err = self.result.ERROR_COMMAND_CONFLICT
            Logger.error(
                "\n\n\n COMMAND CONFLICT!!! \n Waiting for a new command \n\n\n")
        else:
            self.err = self.result.ERROR_UNKNOWN
            Logger.error("Unknown Error --- {}".format(error_type))

        # send result
        self._sendResult(success=False,
                         error=self.err,
                         command=self.current_command)

    #➤ ➤ ➤ ➤ ➤ ERROR: loop
    def on_loop_error(self):
        # send result multiple times (necessary? it seem that sometimes it's not received)
        # self._sendResult(success=False,
        #                  error=self.err,
        #                  command=self.current_command)

        if not self.in_allarm:
            self.idle()


# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇    ROBOT SM CLASS    ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇
# ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇


class RobotMotionSM():

    def __init__(self, robot_name="",  action_server=None):

        self.robot_name = robot_name
        self.robot = RobotMarket.createRobotByName(self.robot_name)

        # CREATE STATE MACHINE
        self.robotSFM = SFMachineRobotDefinition(self.robot)
        self.sfm = machines.SFMachine(name="sfm_" + self.robot_name,
                                      model=self.robotSFM)

        # DEFINE SM STATES
        self.sfm.addState("start")
        self.sfm.addState("idle")
        self.sfm.addState("preparing")
        self.sfm.addState("sendcommand")
        self.sfm.addState("checkcommand")
        self.sfm.addState("generatetrajectory")
        self.sfm.addState("checktrajectory")
        self.sfm.addState("ready")
        self.sfm.addState("moving")
        self.sfm.addState("steady")
        self.sfm.addState("error")

        # DEFINE SM TRANSITIONS
        # start ...
        self.sfm.addTransition("prepare", "start", "preparing")
        self.sfm.addTransition("sendcommand", "start", "sendcommand")
        # idle ...
        self.sfm.addTransition("sendcommand", "idle", "sendcommand")
        # preparing ...
        self.sfm.addTransition("idle", "preparing", "idle")
        # sendcommand ...
        self.sfm.addTransition("checkcommand", "sendcommand", "checkcommand")
        # checkcommand ...
        self.sfm.addTransition("generatetrajectory", "checkcommand", "generatetrajectory")
        # generatetrajectory ...
        self.sfm.addTransition("checktrajectory", "generatetrajectory", "checktrajectory")
        # checktrajectory ...
        self.sfm.addTransition("ready", "checktrajectory", "ready")
        self.sfm.addTransition("steady", "checktrajectory", "steady")
        # ready ...
        self.sfm.addTransition("moving", "ready", "moving")
        self.sfm.addTransition("steady", "ready", "steady")
        # moving ...
        self.sfm.addTransition("steady", "moving", "steady")
        self.sfm.addTransition("sendcommand", "moving", "sendcommand")
        # steady ...
        self.sfm.addTransition("idle", "steady", "idle")
        # ... in error
        self.sfm.addTransition("error", "start", "error")
        self.sfm.addTransition("error", "idle", "error")
        self.sfm.addTransition("error", "preparing", "error")
        self.sfm.addTransition("error", "checkcommand", "error")
        self.sfm.addTransition("error", "sendcommand", "error")
        self.sfm.addTransition("error", "generatetrajectory", "error")
        self.sfm.addTransition("error", "checktrajectory", "error")
        self.sfm.addTransition("error", "ready", "error")
        self.sfm.addTransition("error", "moving", "error")
        self.sfm.addTransition("error", "steady", "error")
        self.sfm.addTransition("idle", "error", "idle")
        self.sfm.addTransition("error", "error", "error")

        self.sfm.create()
        self.sfm.set_state("start")
        Logger.log("\n\n ************* SFM ready to start ***********")

    def registerFeedbackCallback(self, callback):
        self.sfm.getModel().feedback_ext_callback = callback

    def registerResultCallback(self, callback):
        self.sfm.getModel().result_ext_callback = callback

    def prepare(self):  # TODO
        # self.sfm.getModel().prepare()
        pass

    def start(self, robot_command):
        self.robotSFM.uploadCurrentCommand(robot_command)
        if self.sfm.getModel().state != "error":
            try:
                self.sfm.getModel().sendcommand()
                Logger.log("\n\n ************* SFM start ***********")
            except:
                self.sfm.getModel().error("transition_error")
        else:
            error_code = self.sfm.getModel().err
            self.sfm.getModel()._sendResult(success=False,
                                            error=error_code,
                                            command=self.sfm.getModel().current_command)
            Logger.error("\n\n ************* persist error {} *********** \n\n".format(error_code))

    def stepForward(self):
        """ Moves one step forward (1 cycle) the robot's state machine  """
        try:
            self.sfm.loop()
            if self.robotSFM.isTrajectoryAdmissible() and self.sfm.getModel().state == "moving":
                self.robotSFM.applyTrajectory()
        except Exception as e:
            Logger.warning("Exception: {}".format(e))
