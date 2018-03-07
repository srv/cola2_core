#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>Node to keep track of the cola2 running time, used by the safety supervisor to check it against the safety timeout.
<@@
"""

import rospy
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Int32
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper

class Watchdog(object):
    """ This node keeps track of the cola2 running time. Counts the elapsed time since the architecture started
        (actually since the start of this node) or since the last time reset, which can be done through a provided
        service. Publishes the current elapsed time. """


    def __init__(self, name):
        """ Constructor """
        self.name = name

        # Initial time
        self.init_time = rospy.Time.now().to_sec()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Publisher
        resolved_name = rospy.get_name()
        self.pub_elapsed_time = rospy.Publisher(resolved_name + "/elapsed_time", Int32, queue_size = 2)

        # Create reset timeout service
        self.reset_timeout_srv = rospy.Service(resolved_name + '/reset_timeout', Empty, self.reset_timeout)

        # Timer to publish time since init (or last time reset)
        rospy.Timer(rospy.Duration(1.0), self.check_timeout)

        # Show message
        rospy.loginfo("%s: initialized", self.name)


    def check_timeout(self, event):
        """ Callback of the watchdog timer """
        self.diagnostic.add("elapsed_time", str(rospy.Time.now().to_sec() - self.init_time))
        self.diagnostic.setLevel(DiagnosticStatus.OK)

        # Publish elapsed time
        msg = Int32()
        msg.data = int(rospy.Time.now().to_sec() - self.init_time)
        self.pub_elapsed_time.publish(msg)

    def reset_timeout(self, req):
        """ Service used to reset timeout """
        rospy.loginfo("%s: Reset Timeout!", self.name)
        self.init_time = rospy.Time.now().to_sec()
        return EmptyResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('cola2_watchdog')
        watchdog = Watchdog(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
