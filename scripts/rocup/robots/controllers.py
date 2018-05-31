import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import PyKDL
import tf
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import superros.transformations as transformations
from superros.logger import Logger

from rocup.srv import IKService, IKServiceResponse
from rocup.msg import RobotFollowStatus


""" GLOBAL PARAMETERS """
ROBOT_CONTROLLER_PUBLISHER_NAME = "joint_states"
ROBOT_CONTROLLER_SUBSCRIBER_NAME = "joint_command"
ROBOT_CONTROLLER_REPLACE_ROBOT_NAME_PREFIX_IN_JOINTS = True
ROBOT_CONTROLLER_MOVING_POSITION_EPSILON = 0.001
ROBOT_CONTROLLER_MOVING_ROTATION_EPSILON = 0.002
ROBOT_DEFAULT_IK_SERVICE_SUFFIX = "/ik_service_node/ik_service"
ROBOTS_DEFAULT_WORLD_NAME = "world"
ROBOTS_DEBUG_MODE = True


class GeneralIKService:
    """ General Service Wrapper for IKService messages """

    def __init__(self, service_name, service_manifest=IKService, timeout=5.0):
        self.service_name = service_name
        self.service_manifest = service_manifest
        self.active = False
        Logger.log("GeneralIKService: waiting for service " +
                   service_name + "...")

        try:
            rospy.wait_for_service(self.service_name, timeout=timeout)
        except rospy.ROSException as e:
            Logger.error("Service timeout:" + str(e))
            return
        self.service = rospy.ServiceProxy(
            self.service_name, service_manifest)
        self.active = True
        Logger.log("Service Connected!")

    def isActive(self):
        return self.active

    def simple_ik(self, target_frame, q_current, ik_type=0):
        """ Calls a simple_ik service """
        try:
            p = Pose()
            p.position.x = target_frame.p.x()
            p.position.y = target_frame.p.y()
            p.position.z = target_frame.p.z()
            qu = target_frame.M.GetQuaternion()
            p.orientation.x = qu[0]
            p.orientation.y = qu[1]
            p.orientation.z = qu[2]
            p.orientation.w = qu[3]
            q_in = Float64MultiArray()
            q_in.data = q_current
            response = self.service(p, q_in, 0)

            return response
        except rospy.ServiceException, e:
            Logger.error("GeneralIKService: Service call failed: % s" % e)


class RobotController:
    """ This class is used as interface for JointState/Command Topics of target robot """

    def __init__(self, robot_name, joint_names, configuration=None):
        self.robot_name = robot_name
        self.joint_names = joint_names
        self.configuration = configuration
        self.q_map = {}
        self.q_dot_map = {}
        for n in joint_names:
            self.q_map[n] = 0.0
            self.q_dot_map[n] = 0.0
        if configuration == None:
            self.joint_state_topic = "/" + robot_name + \
                "/" + ROBOT_CONTROLLER_PUBLISHER_NAME
            self.joint_command_topic = "/" + robot_name + \
                "/" + ROBOT_CONTROLLER_SUBSCRIBER_NAME
        else:
            self.joint_state_topic = configuration["joint_state_topic"]
            self.joint_command_topic = configuration["joint_command_topic"]
        self.joint_command_pub = rospy.Publisher(
            self.joint_command_topic, JointState, queue_size=1)
        self.joint_command_sub = rospy.Subscriber(
            self.joint_state_topic, JointState, self._jointStateCallback)
        self.void_message = self.buildJointMessage()

    def buildJointMessage(self):
        """ Creates a void JointState message filling it with Joint Names """
        joint_msg = JointState()
        joint_msg.name = self.joint_names
        joint_msg.position = []
        for name in joint_msg.name:
            joint_msg.position.append(0.0)
        return joint_msg

    def jointCommand(self, q, time=rospy.Time(0)):
        """ Send a Q JointCommand reference to target topic """
        self.void_message.position = q
        self.void_message.header.stamp = time
        self.joint_command_pub.publish(self.void_message)

    def getQ(self):
        """ Retrieves the current state of Q Joints """
        q = []
        for n in self.joint_names:
            q.append(self.q_map[n])
        return np.array(q)

    def getQDot(self):
        """ Retrieves the current state of Q_dot Joints """
        q = []
        for n in self.joint_names:
            q.append(self.q_dot_map[n])
        return np.array(q)

    def isMoving(self, epsilon=ROBOT_CONTROLLER_MOVING_ROTATION_EPSILON):
        """ Checks if robot is moving comparing actual Q_dot norm with EPS """
        v = np.linalg.norm(self.getQDot())
        return v > epsilon

    def _jointStateCallback(self, msg):
        """ Internal callback invoked every new JointState message from topic """
        for i, n in enumerate(msg.name):
            if ROBOT_CONTROLLER_REPLACE_ROBOT_NAME_PREFIX_IN_JOINTS:
                n = n.replace(self.robot_name + "/", "")
            if len(msg.position) == len(msg.name):
                self.q_map[n] = msg.position[i]
            if len(msg.velocity) == len(msg.name):
                self.q_dot_map[n] = msg.velocity[i]

    def applyShape(self, robot_shape, time=rospy.Time(0)):
        """ Applies a Shape to robot """
        if robot_shape:
            self.jointCommand(robot_shape.q, time)


class RobotShape(object):
    """ A Robot shape is just a joint configuration with a name """

    def __init__(self, name, q):
        self.name = name
        self.q = q


class RobotStatus(object):
    """ This class is used as interface for JointState/Command Topics of target robot """

    def __init__(self, name):
        self.robot_name = name
        follow_status_topic = "/" + self.robot_name + "/" + "follow_status"
        self.follow_status_sub = rospy.Subscriber(
            follow_status_topic, RobotFollowStatus, self._followStatusCallback)
        self.alarm = False
        self.moving = False
        self.tool = PyKDL.Frame()

    def _followStatusCallback(self, msg):
        """ Internal callback invoked every new RobotFollowStatus message from topic """
        self.alarm = msg.alarm
        self.moving = msg.moving
        tool_msg_p = msg.tool.position
        tool = PyKDL.Frame(PyKDL.Vector(tool_msg_p.x,
                                        tool_msg_p.y,
                                        tool_msg_p.z))
        tool_msg_q = msg.tool.orientation
        tool.M = PyKDL.Rotation.Quaternion(tool_msg_q.x,
                                           tool_msg_q.y,
                                           tool_msg_q.z,
                                           tool_msg_q.w)
        self.tool = tool
        # print(self.tool)


class Robot(object):
    """ Class defining a Robot structure, with Controller, IK Services and more """

    def __init__(self, configuration, tf_listener=None, tf_broadcaster=None):
        self.robot_name = configuration["robot_name"]
        self.joint_names = configuration["joint_names"]
        self.link_names = configuration["link_names"]
        self.std_tools = configuration["tools"]
        self.parameters = configuration["parameters"]
        self.robot_shapes = {}
        self.robot_status = RobotStatus(self.robot_name)
        if "joint_topics" in configuration.keys():
            self.joint_topics = configuration["joint_topics"]
        else:
            self.joint_topics = None
        self.robot_controller = RobotController(
            self.robot_name, self.joint_names, configuration=self.joint_topics)
        self.tf_listener = tf_listener
        self.tf_broadcaster = tf_broadcaster
        if self.tf_listener == None:
            self.tf_listener = tf.TransformListener()
        if self.tf_broadcaster == None:
            self.tf_broadcaster = tf.TransformBroadcaster()
        if configuration["auto_ik_service"]:
            self.ik_service_name = "/" + self.robot_name + ROBOT_DEFAULT_IK_SERVICE_SUFFIX
            self.ik_service = GeneralIKService(self.ik_service_name)
        else:
            self.ik_service = None

    def getName(self):
        return self.robot_name

    def getVoidQ(self):
        q = []
        for i in range(0, len(self.joint_names)):
            q.append(0.0)
        return q

    def addRobotShape(self, robot_shape):
        """ Adds a Robot Shape """
        self.robot_shapes[robot_shape.name] = robot_shape

    def getShapeQ(self, shape_name):
        """ returns q coordinates of a Robot Shape """
        if self.robot_shapes.get(shape_name):
            return self.robot_shapes.get(shape_name).q
        return []

    def shape(self, shape_name, time=rospy.Time(0)):
        """ Applies a Shape to robot  """
        if self.robot_shapes.get(shape_name):
            self.getController().applyShape(self.robot_shapes.get(shape_name), time)
        else:
            Logger.warning("No shape with name '{}' for robot '{}'".format(
                shape_name, self.robot_name))

    def getController(self):
        """ returns RobotController """
        return self.robot_controller

    def getLinkName(self, index):
        """ Gets link name by index """
        if index == -1:
            index = len(self.link_names) - 1
        if index >= 0 and index < len(self.link_names):
            return self.robot_name + "/" + self.link_names[index]
        else:
            return ""

    def isIKServiceActive(self):
        """ Returns TRUE if an IK Service is available """
        return self.ik_service != None

    def getIKService(self):
        """ Gets the IK Service """
        return self.ik_service

    def getBaseFrame(self, time=rospy.Time(0)):
        """ Retrieves the First Link Frame of the Robot, may return Identity if problems occur """
        base_frame = transformations.retrieveTransform(
            self.tf_listener, ROBOTS_DEFAULT_WORLD_NAME, self.getLinkName(0), time, print_error=ROBOTS_DEBUG_MODE)
        return base_frame

    def getEEFrame(self, time=rospy.Time(0), world_frame=False):
        """ Retrieves the Last Link Frame of the Robot, may return Identity if problems occur """
        if world_frame:
            ee_frame = transformations.retrieveTransform(
                self.tf_listener, ROBOTS_DEFAULT_WORLD_NAME, self.getLinkName(-1), time, print_error=ROBOTS_DEBUG_MODE)
        else:
            ee_frame = transformations.retrieveTransform(
                self.tf_listener, self.getLinkName(0), self.getLinkName(-1), time, print_error=ROBOTS_DEBUG_MODE)
        return ee_frame

    def publishTool(self, tool=None, name="current", time=0):
        """ Publish of Tool relative to EE Frame """
        if not tool:
            tool = self.getCurrentTool()
        tool_name = self.robot_name + "/tool"
        transformations.broadcastTransform(
            self.tf_broadcaster, tool, tool_name, self.getLinkName(-1), time)

    def getCurrentTool(self):
        """ Gets the current PyKDL Tool Frame. """
        return self.robot_status.tool

    def getParam(self, name):
        return self.parameters[name]
