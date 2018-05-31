import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from rocup.msg import UiEvent
from superros.logger import Logger
import StringIO
import struct
import collections


class MessageStorage(object):

    def __init__(self):
        Logger.log("Connecting with MongoDB Database...")
        self.msg_store = MessageStoreProxy()
        Logger.log("Connected!")

    def store(self, name, message):
        try:
            return self.msg_store.insert_named(name, message)
        except rospy.ServiceException, e:
            Logger.error(e)
            return None

    def replace(self, name, message, meta={}):
        try:
            self.msg_store.update_named(name, message, upsert=True, meta=meta)
        except rospy.ServiceException, e:
            Logger.error(e)
            return None
  

    def searchByName(self, name, message_type, single=True):
        try:
            return self.msg_store.query(
                message_type._type,
                {"_meta.name": {'$regex': name}},
                single=single)
        except rospy.ServiceException, e:
            Logger.error(e)
            return None

    def serachByCustomMeta(self, key, value, message_type):
        try:
            return self.msg_store.query(
                message_type._type,
                {"_meta.{}".format(key): {'$regex': value}},
                single=False)
        except rospy.ServiceException, e:
            Logger.error(e)
            return None

    def deleteByID(self, id):
        try:
            self.msg_store.delete(id)
        except rospy.ServiceException, e:
            Logger.error(e)

    def deleteByName(self, name, message_type):
        query = self.searchByName(name, message_type, single=False)
        if query:
            for item in query: 
                try:
                    self.msg_store.delete(item.id)
                except rospy.ServiceException, e:
                    Logger.error(e)
