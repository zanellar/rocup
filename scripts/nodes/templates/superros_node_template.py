#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from superros.comm import RosNode

#⬢⬢⬢⬢⬢➤ NODE
node = RosNode("tf_export_test")

node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))


while node.isActive():

    node.tick()
