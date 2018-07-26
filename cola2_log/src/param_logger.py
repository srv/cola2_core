#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""@@>Publishes all parameters in a topic for logging/debugging purposes.<@@"""

import os
import rospy
import rosparam
from std_msgs.msg import String


class ParamLoggerNode(object):
    """Log all parameters in rosparam to a topic or to a file."""

    def __init__(self):
        """Constructor."""
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
    # init node
    ParamLoggerNode()
    # keep running
    rospy.spin()
