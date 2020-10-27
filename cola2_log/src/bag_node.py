#!/usr/bin/env python3
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>Provides services to run and stop the launch file launch_bag.launch.<@@
"""

import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import subprocess
import os
import time

class LogBag:
    """LogBag class."""

    def __init__(self):
        """Class constructor."""
        self.start_bag = rospy.Service('~enable_logs', Trigger, self.enable_logs)
        self.stop_bag = rospy.Service('~disable_logs', Trigger,
                                      self.disable_logs)
        self.bag_enabled = False
        rospy.loginfo("This node will execute the launch file cola2_log/launch/bag.launch")

        # namespace = robot name
        ns = rospy.get_namespace()
        if ns[0] == '/':
            ns = ns[1:]
        if ns[-1] == '/':
            ns = ns[:-1]
        self.robot_name = ns

    def enable_logs(self, req):
        """Run launch/bag.launch file."""
	# Check if bags folder exists
        home_path = os.path.expanduser('~')
        bags_path = os.path.join(home_path, 'bags')
        if not os.path.isdir(bags_path):
            os.makedirs(bags_path)

        # Subprocess
        command = ("roslaunch cola2_" + self.robot_name + " bag.launch robot_name:={:s}").format(self.robot_name)
        subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd="./")
        self.bag_enabled = True
        rospy.loginfo("Launched bag file")
        tr = TriggerResponse()
        tr.success = True
        tr.message = "Launched bag file"
        return tr

    def disable_logs(self, req):
        """Stop launch/bag.launch file."""
        tr = TriggerResponse()
        if self.bag_enabled:
            nodes = subprocess.check_output(['rosnode', 'list']).split()
            for node in nodes:
                if "record_auv" in node:
                    subprocess.call(['rosnode', 'kill', node])
                    time.sleep(1.0)

            # killcommand = "kill -9 " + str(self.pid+1)
            # subprocess.Popen(killcommand, shell=True)
            # rospy.loginfo("Kill process: " + killcommand)
            self.bag_enabled = False
            tr.success = True
            tr.message = "Stopped bag file"
        else:
            rospy.logwarn("Bag is not enabled!")
            tr.success = False
            tr.message = "Bag is not enabled"

        return tr

if __name__ == '__main__':
    """ Main function. """
    rospy.init_node('bag_node')
    log = LogBag()
    rospy.spin()
