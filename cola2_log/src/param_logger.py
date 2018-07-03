#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""@@>Publishes all parameters in a topic for logging/debugging purposes.<@@"""

import rospy
import rosparam
from std_msgs.msg import String
import os
import sys


class ParamLoggerNode(object):
    """Log all parameters in rosparam to a topic or to a file."""

    def __init__(self, path):
        """Constructor."""
        # save path
        self.path = path
        rospy.loginfo("save params path: " + self.path)
        # init node
        rospy.init_node('param_logger')
        # publisher, timer and service
        self.pub = rospy.Publisher('~params_string', String, queue_size=1, latch=True)
        self.tim = rospy.Timer(rospy.Duration(10.0 * 60.0), self.callback)
        # call it once at begining
        self.callback(None)

    def callback(self, dummy_event):
        """Callback to collect all parameters."""
        # Dump to temp file
        rosparam.dump_params("temp.yaml", "/")
        # Read file into a string message
        msg = String()
        msg.data = open("temp.yaml").read()
        self.pub.publish(msg)
        # Delete temp file
        os.remove("temp.yaml")


if __name__ == '__main__':
    # check imput arguments
    path = "."
    if len(sys.argv) > 1:
        path = sys.argv[1]
    # init node
    ParamLoggerNode(path)
    # keep running
    rospy.spin()
