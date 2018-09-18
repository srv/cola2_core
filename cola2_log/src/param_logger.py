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
from std_srvs.srv import Trigger, TriggerResponse


class ParamLoggerNode(object):
    """Log all parameters in rosparam to a topic or to a file."""

    def __init__(self):
        """Constructor."""
        # init node
        rospy.init_node('param_logger')
        # publisher and service
        self.pub = rospy.Publisher('~params_string', String, queue_size=1, latch=True)
        self.srv = rospy.Service('~publish_params', Trigger, self.srv_publish)
        # call it once at begining
        self.publish()

    def publish(self):
        """Publish all collected parameters."""
        # Dump to temp file
        rosparam.dump_params("temp.yaml", "/")
        # Read file into a string message
        msg = String()
        msg.data = open("temp.yaml").read()
        self.pub.publish(msg)
        # Delete temp file
        os.remove("temp.yaml")

    def srv_publish(self, req):
        """Publish params when service is called."""
        # debug info
        srv = req._connection_header['service']
        who = req._connection_header['callerid']
        rospy.loginfo("service: '{:s}' called from '{:s}'".format(srv, who))
        # publish
        self.publish()
        # return response
        return TriggerResponse(True, "parameters published")


if __name__ == '__main__':
    # init node
    ParamLoggerNode()
    # keep running
    rospy.spin()
