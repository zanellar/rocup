#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode
from std_msgs.msg import String


def topic_collback(msg):
    print msg.data


#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("tf_export_test")

node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))

topic_pub = node.createPublisher("/topic_1", String)
node.createSubscriber("/topic_2", String, topic_collback)

while node.isActive():

    msg = String()
    msg.data = "hello world"
    topic_pub.publish(msg)

    node.tick()
