import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random
import PyKDL
import tf
import sys
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from rocup.msg import RobotFollow
import superros.transformations as transformations
import rocup.robots.controllers as controllers
from superros.logger import Logger
from rocup.srv import IKService, IKServiceResponse
from rocup.proxy.target_follower_proxy import TargetFollowerProxy

""" GLOBAL PARAMETERS """


class TrajectoriesMath(object):

    @staticmethod
    def qLinearInterpolation(q1, q2, steps):

        vq1 = np.array(q1)
        vq2 = np.array(q2)

        dq_temp = np.array([])
        for i in range(0, len(q1)):
            if steps <= 1:
                q = [q2[i]]
            else:
                q = np.linspace(q1[i], q2[i], num=steps, endpoint=True)

            if len(dq_temp) == 0:
                dq_temp = q
            else:
                dq_temp = np.vstack([dq_temp, q])

        columns = []
        for c in range(0, dq_temp.shape[1]):
            col = dq_temp[:, c]
            columns.append(col)

        return columns

    @staticmethod
    def framesLinearInterpolation(frame1, frame2, steps):
        x = np.linspace(frame1.p.x(), frame2.p.x(), num=steps, endpoint=True)
        y = np.linspace(frame1.p.y(), frame2.p.y(), num=steps, endpoint=True)
        z = np.linspace(frame1.p.z(), frame2.p.z(), num=steps, endpoint=True)

        q1 = frame1.M.GetQuaternion()
        q2 = frame2.M.GetQuaternion()

        qx = np.linspace(q1[0], q2[0], num=steps, endpoint=True)
        qy = np.linspace(q1[1], q2[1], num=steps, endpoint=True)
        qz = np.linspace(q1[2], q2[2], num=steps, endpoint=True)
        qw = np.linspace(q1[3], q2[3], num=steps, endpoint=True)

        frames = []
        for i in range(0, len(x)):
            f = PyKDL.Frame()
            f.M = PyKDL.Rotation.Quaternion(
                qx[i],
                qy[i],
                qz[i],
                qw[i]
            )
            f.p.x(x[i])
            f.p.y(y[i])
            f.p.z(z[i])
            frames.append(f)
        return frames


class TrajectoryPoint(object):
    """ Trajectory Base element """

    def __init__(self, frame=PyKDL.Frame(), q=0.0, q_dot=0.0, invalid=True):
        self.frame = frame
        self.q = q
        self.q_dot = q_dot
        self.invalid = invalid

    def isValid(self):
        return not self.invalid


class FuzzyTrajectoryGenerator(object):
    """ Very simple Trajectory Generator """
    STATUS_OK = 1
    STATUS_INVALID_POINTS = 10
    STATUS_INVALID_SPEED = 20

    def __init__(self, robot):
        self.robot = robot
        self.robot_name = robot.getName()

        self.target_follower = TargetFollowerProxy(self.robot_name)

        self.checkIKService()
        self.current_trajectory = []
        self.current_trajectory_length = -1
        self.target_q = []
        self.initial_q = []
        self.status = FuzzyTrajectoryGenerator.STATUS_OK

    def setCurrentTrajectory(self, traj):
        self.current_trajectory = traj
        self.current_trajectory_length = len(traj)

        if self.current_trajectory_length > 0:
            target_point = traj[len(traj) - 1]
            initial_point = traj[0]
            self.target_q = target_point.q
            self.initial_q = initial_point.q
        else:
            self.target_q = []
            self.initial_q = []

    def getCurrentTrajectoryPercentage(self):
        return 1.0 - float(len(self.current_trajectory)) / float(self.current_trajectory_length)

    def checkIKService(self):
        if not self.robot.isIKServiceActive():
            Logger.error(
                "Robot '{}' has no IK Service active!".format(self.robot.getName()))
            return False
        return True

    def generateCartesianFrames(self, frame1, frame2, hz, time):
        steps = time * hz
        cartesian_interpolation = TrajectoriesMath.framesLinearInterpolation(
            frame1, frame2, steps)
        return cartesian_interpolation

    def generateTrajectoryFromShapes(self, q_in, q_out, hz, time):
        """ Generates a linear trajectory between two jointconfigurations """
        dq = TrajectoriesMath.qLinearInterpolation(q_in, q_out, hz * time)
        trajectory = []
        for q in dq:
            trajectory_point = TrajectoryPoint(q=q, q_dot=0.0, invalid=False)
            trajectory.append(trajectory_point)
        self.setCurrentTrajectory(trajectory)
        self.status = FuzzyTrajectoryGenerator.STATUS_OK
        # print("trjectory len {}".format(len(trajectory)))
        return trajectory

    def generateTrajectoryFromFrames(self, q_in, frame1, frame2, hz, time):
        """ Generates a linear trajectory between two cartesian frames """
        cartesian_trajectory = self.generateCartesianFrames(frame1, frame2, hz, time)

        trajectory_points = []
        if not self.checkIKService():
            self.setCurrentTrajectory(trajectory_points)
            return trajectory_points
        else:
            current_q = q_in
            last_q = q_in
            success_counter = 0.0

            for frame in cartesian_trajectory:
                trajectory_point = TrajectoryPoint(frame, last_q, 0.0, invalid=True)
                response = self.robot.getIKService().simple_ik(frame, last_q)
                if response.status == IKServiceResponse.STATUS_OK:
                    trajectory_point.q = response.q_out.data
                    trajectory_point.invalid = False
                    success_counter += 1.0
                trajectory_points.append(trajectory_point)
                last_q = trajectory_point.q
            success_counter = success_counter / float(len(cartesian_trajectory))
            Logger.log("Trajectory Computation Success {}%".format(success_counter * 100))

            if success_counter > 0.99:
                self.status = FuzzyTrajectoryGenerator.STATUS_OK
            else:
                self.status = FuzzyTrajectoryGenerator.STATUS_INVALID_POINTS

            self.setCurrentTrajectory(trajectory_points)
            return trajectory_points

    def generateTrajectoryFreeFromFrames(self, q_in, frame1, frame2, steps=1, middle_frames=1, number_of_tries=5,  agumented_middle_joints=True,  perturbate_middle_frames=True):
        perturbation_gain = 0
        p1 = transformations.KDLVectorToList(frame1.p)
        p2 = transformations.KDLVectorToList(frame2.p)
        dist_frames = np.linalg.norm(np.subtract(p1, p2))
        while number_of_tries > 0:
            if middle_frames > 1:
                cartesian_trajectory = self.generateCartesianFrames(frame1, frame2, middle_frames, 1)
            else:
                cartesian_trajectory = [frame2]
            number_of_tries -= 1

            trajectory_points = []
            if not self.checkIKService():
                self.setCurrentTrajectory(trajectory_points)
                return trajectory_points
            else:
                current_q = q_in
                last_q = q_in
                success_counter = 0.0

                for frame in cartesian_trajectory:
                    if perturbate_middle_frames and frame != cartesian_trajectory[-1]:  # we perturbate the frame hoping to avoid the singularities in the IK calculation
                        frame = FuzzyTrajectoryGenerator._perturbateFrameRotation(frame, perturbation_gain)
                    trajectory_point = TrajectoryPoint(frame, last_q, 0.0, invalid=True)
                    response = self.robot.getIKService().simple_ik(frame, last_q)
                    if response.status == IKServiceResponse.STATUS_OK:
                        trajectory_point.q = response.q_out.data
                        trajectory_point.invalid = False
                        success_counter += 1.0
                    last_q = trajectory_point.q
                success_counter = success_counter / float(len(cartesian_trajectory))
                Logger.log("IK Computation (#{} segments) Success {}%".format(len(cartesian_trajectory), success_counter * 100))
                if success_counter > 0.99:
                    inc_steps = np.linalg.norm(np.arccos(np.cos(np.subtract(q_in, last_q))))
                    q_trajectory_steps = steps + int(agumented_middle_joints * inc_steps * 500)
                    return self.generateTrajectoryFromShapes(q_in, last_q, q_trajectory_steps, 1)
                    # self.setCurrentTrajectory(trajectory_points)
                    self.status = FuzzyTrajectoryGenerator.STATUS_OK
                    # return trajectory_points
                else:
                    perturbation_gain = 1
                    middle_frames *= 5
        Logger.warning("Fail IK")
        self.status = FuzzyTrajectoryGenerator.STATUS_INVALID_POINTS
        trajectory_points = []
        self.setCurrentTrajectory(trajectory_points)

        return trajectory_points

    @staticmethod
    def _perturbateFrameRotation(frame, gain=1.0):
        base_angle = math.pi / 12.0
        d1 = gain * random.random() * base_angle
        d2 = gain * random.random() * base_angle
        d3 = gain * random.random() * base_angle
        newframe = PyKDL.Frame()
        newframe.p = frame.p
        newframe.M = frame.M.EulerZYX(d1, d2, d3)
        return newframe

    def applyTrajectory(self, time):
        if len(self.current_trajectory) > 0:
            point = self.current_trajectory.pop(0)
            if point.isValid():
                sys.stdout.write("Remaining Trajectory Points: {} \r".format(len(self.current_trajectory)))
                sys.stdout.flush()
                self.target_follower.setTarget(target=list(point.q),
                                               target_type=TargetFollowerProxy.TYPE_JOINTS,
                                               target_source=TargetFollowerProxy.SOURCE_TRAJECTORY)

    def isTrajectoryAvailable(self):
        if self.current_trajectory is None:
            return False
        else:
            if len(self.current_trajectory) <= 0:
                return False
            else:
                return True

    def getTargetQ(self):
        return np.array(self.target_q)

    def getInitialQ(self):
        return np.array(self.initial_q)

    def computationSuccess(self):
        return self.status == FuzzyTrajectoryGenerator.STATUS_OK

    def clearTrajectory(self):
        self.current_trajectory = []
