#!/usr/bin/env python
# Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

import os
import signal
import datetime
import subprocess

import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from diagnostic_msgs.msg import DiagnosticStatus

from cola2_ros.diagnostic_helper import DiagnosticHelper

class LogBag:
    """LogBag class."""

    def __init__(self):
        """Class constructor."""

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper("bag_recorder", rospy.get_name())
        self.diagnostic.set_enabled(True)

        # Create services
        self.start_bag = rospy.Service('~enable_logs', Trigger, self.enable_logs)
        self.stop_bag = rospy.Service('~disable_logs', Trigger, self.disable_logs)

        # Initialize dictionary that will contain the stack of subprocesses
        self.launch_number = 0
        self.stacks = {}

        # Obtain robot name from the namespace
        ns = rospy.get_namespace()
        if ns[0] == '/':
            ns = ns[1:]
        if ns and ns[-1] == '/':
            ns = ns[:-1]
        self.robot_name = ns

        # Get launched sensors
        self.sensors = {}
        sensor_names = rospy.get_param('sensors', [])
        for sensor_name in sensor_names:
            print(sensor_name)
            service_name = sensor_name + '/reload_params'
            self.sensors[sensor_name] = rospy.ServiceProxy(service_name, Trigger)
            rospy.wait_for_service(service_name, 2)

        # Start timer
        rospy.Timer(rospy.Duration(1), self.diagnostics_timer)

        rospy.loginfo("Initialized. This node will execute the launch file bag.launch")

    def enable_logs(self, req):
        """Run launch/bag.launch file."""

        # Check if datasets folder exists
        home_path = os.path.expanduser('~')
        datasets_path = os.path.join(home_path, 'datasets')
        if not os.path.isdir(datasets_path):
            os.makedirs(datasets_path)
        
        # Get day and hour
        now = datetime.datetime.now()
        full_day = now.strftime("%Y_%m_%d")
        full_hour = now.strftime("%H_%M_%S")

        # Check if day and hour folders exists
        datasets_day_hour_path = os.path.join(datasets_path, full_day, full_hour)
        if not os.path.isdir(datasets_day_hour_path):
            os.makedirs(datasets_day_hour_path)

        # Create launched sensors folder
        for sensor in self.sensors.keys():
            sensor_path = os.path.join(datasets_day_hour_path, sensor)
            if not os.path.isdir(sensor_path):
                os.makedirs(sensor_path)
            rospy.set_param(os.path.join(sensor, 'store_path'), sensor_path)
            self.sensors[sensor]()

        # Create bags folder
        bags_path = os.path.join(datasets_day_hour_path, 'bags')
        if not os.path.isdir(bags_path):
            os.makedirs(bags_path)

        # Get caller id
        caller_id = req._connection_header['callerid']

        # Start launch file subprocess
        command = ("roslaunch cola2_sparus2 bag.launch robot_name:={:s} launch_number:={:s} output_path:={:s}").format(self.robot_name, str(self.launch_number), bags_path + '_' + self.robot_name)
        self.launch_number = self.launch_number + 1
        #subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd="./")
        pro = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd="./", preexec_fn=os.setsid)

        # Add subprocess to the stack associated with the caller id
        if caller_id in self.stacks:
            self.stacks[caller_id].append(pro)
        else:
            self.stacks[caller_id] = [pro]

        tr = TriggerResponse()
        tr.success = True
        tr.message = "Start logging .bag file"
        rospy.loginfo(tr.message)
        return tr

    def disable_logs(self, req):
        """Stop launch/bag.launch file."""

        # Get caller id
        caller_id = req._connection_header['callerid']

        tr = TriggerResponse()
        if caller_id in self.stacks:
            # There are bags enabled for this caller id, so stop the last one
            os.killpg(os.getpgid(self.stacks[caller_id].pop().pid), signal.SIGTERM)
            if len(self.stacks[caller_id]) == 0:
                self.stacks.pop(caller_id, None)  # Remove key if the stack list is empty
            else:
                rospy.loginfo("Stopping .bag file but other .bag files are still active for id " + caller_id)
            tr.success = True
            tr.message = "Stopped logging .bag file"
            rospy.loginfo(tr.message)
        else:
            # This caller id has no bags associated, so nothing to do
            tr.success = False
            tr.message = "Impossible to stop logging .bag file. No active .bag files for id " + caller_id
            rospy.logwarn(tr.message)

        return tr

    def stop_all(self):
        # When this node dies, it stops all the bags it has started
        for caller_id in self.stacks:
            while len(self.stacks[caller_id]) > 0:
                print("Disabling bag for id " + caller_id)  # The rosout may not work at this point, so better use print
                os.killpg(os.getpgid(self.stacks[caller_id].pop().pid), signal.SIGTERM)

    def diagnostics_timer(self, event):
        """ Callback from the diagnostics timer """
        self.diagnostic.set_level_and_message(DiagnosticStatus.OK)
        self.diagnostic.report_valid_data(event.current_real)
        self.diagnostic.publish(event.current_real)


if __name__ == '__main__':
    """ Main function. """
    rospy.init_node('bag_node')
    log = LogBag()
    rospy.spin()
    log.stop_all()
