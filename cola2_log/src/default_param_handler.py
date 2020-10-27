#!/usr/bin/env python3
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""@@>This node provides services to store current parameters as defaults
writting them to their corresponding .YAML files<@@"""

# ROS imports
import rospy
import roslib
from std_srvs.srv import Trigger, TriggerResponse
from cola2_lib.rosutils import param_loader

# Other imports
import os
import subprocess
from ruamel.yaml import YAML

class DefaultParamHandler(object):
    """
    This node provides services to store current parameters as defaults
    (writting them to their corresponding .YAML files
    """

    def __init__(self, name):
        """ Init the class. """
        self.name = name
        self.get_config()
        self.config_path = roslib.packages.get_pkg_dir(self.config_pkg) + '/' + self.config_folder
        # Service to update parameters from param server to corresponding YAML files
        self.update_yamls_srv = rospy.Service(self.name + '/update_params_in_yamls', Trigger, self.update_params_in_yamls)
        rospy.loginfo("Starting default param handler on config path: {}".format(self.config_path))

    def update_params_in_yamls(self, req):
        """ Callback of the service to to update parameters from param server to corresponding YAML files"""

        # Helper to parse and write yaml files preserving comments, indents, etc.
        yaml = YAML()
        # Flag to preserve quotes in string values
        yaml.preserve_quotes = True
        yaml.default_flow_style = True
        yaml.allow_duplicate_keys = True
        # First get the current params from param server. We do no use the command dump as we want to construct a
        # dictionary [param (with all the sub levels in one line), value]
        dump_params = dict()
        namespace = rospy.get_namespace()
        rospy.loginfo("Getting parameters from param server")
        for p in rospy.get_param_names():
            dump_params[self.remove_namespace(p, namespace)] = rospy.get_param(p)
        rospy.loginfo("Updating parameters from param server to corresponding YAML files")
        
        r = TriggerResponse()
        try:
            # Iterate through all the yaml files in the config folder
            for yamlfile in os.listdir(self.config_path):
                if yamlfile.endswith(".yaml"):
                    filename = os.path.join(self.config_path, yamlfile)
                    fy = open(filename, 'r')
                    yaml_params = yaml.load(fy)
                    # For each param in the file look if the value of the current parameter is different
                    if yaml_params is not None:
                        for k, v in yaml_params.iteritems():
                            if k in dump_params:  # If parameter is in the current set of loaded params
                                if dump_params[k] != yaml_params[k]:
                                    rospy.loginfo("Current value of {} in param server: {}".format(k, dump_params[k]))
                                    rospy.loginfo("Value in {} file: {}".format(filename, yaml_params[k]))
                                    rospy.loginfo("Updating param in yaml file")
                                    yaml_params[k] = dump_params[k]
                        # Close file for reading
                        fy.close()
                        # Open for writing and update all changes
                        fyw = open(filename, 'w')
                        yaml.dump(yaml_params, fyw)
                        fyw.close()
                    else:
                        fy.close()
            r.success = True
            r.message = "Parameters updated correctly to YAML files"
        except:
            r.success = False
            r.message = "Error in dumping parameters to YAML files"
       
        return r


    def remove_namespace(self, text, ns):
        if text.startswith(ns):
            return text[len(ns):]
        return text

    def get_config(self):
        """ Read parameters from ROS param server."""

        param_dict = {'config_pkg':('config_pkg', 'cola2_girona500'),
                      'config_folder':('config_folder', 'config')}

        param_loader.get_ros_params(self, param_dict)


if __name__ == '__main__':
    try:
        rospy.init_node('default_param_handler')
        dph = DefaultParamHandler(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
