#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


# -*- coding: utf-8 -*-

"""@@>Publishes all parameters in a topic for logging/debugging purposes.<@@"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import datetime
import os
import sys

global params_path

params_path = str()

def callback(event):
    """Callback to collect all parameters."""
    strings = list()
    for name in rospy.get_param_names():
        if not 'robot_description' in name:
            if not 'roslaunch/uris' in name:
                value = rospy.get_param(name)
                line = '{:s}={}'.format(name, value)
                strings.append(line)
    msg = String()
    msg.data = '\n'.join(strings)
    pub.publish(msg)

def save_params_callback(req):
    """Service to save all rosparam server param in a file (date_time.yaml)."""
    file_name = datetime.datetime.now().strftime('%Y%m%d%H%M%S') + '_parameters.yaml'
    file_name = params_path + '/' + file_name
    print('Create file: ' + file_name)
    param_file = open(file_name, "w")
    try:
        os.remove(params_path + "/latest_params.yaml")
    except OSError:
        pass
    os.symlink(file_name, params_path + "/latest_params.yaml")
    for name in rospy.get_param_names():
        if not 'robot_description' in name:
            if not 'roslaunch/uris' in name:
                if not 'rosbridge_websocket' in name:
                    if not 'rosapi' in name:
                        value = rospy.get_param(name)
                        if isinstance(value, str):
                            line = "{:s}: '{}'\n".format(name, value)
                        else:
                            line = '{:s}: {}\n'.format(name, value)
                        param_file.write(line)
    param_file.close()
    return EmptyResponse()

if __name__ == '__main__':
    # Init node
    rospy.init_node('param_logger')
    params_path = "."
    if len(sys.argv) > 1:
        params_path = str(sys.argv[1])
    rospy.loginfo("Save params path: " + params_path)
    resolved_name = rospy.get_name()
    pub = rospy.Publisher(resolved_name + '/params_string', String, queue_size=1, latch=True)
    tim = rospy.Timer(rospy.Duration(120), callback, oneshot=True)
    s = rospy.Service('/save_params_to_file', Empty, save_params_callback)
    rospy.spin()
