#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>This node checks if the vehicle moves beyond some given virtual limits defined in NED coordinates.<@@"""
"""
Created on 02/13/2014

@author: Narcis Palomeras
"""

# ROS imports
import roslib
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from diagnostic_msgs.msg import DiagnosticStatus
from dynamic_reconfigure.server import Server

from cola2_msgs.msg import NavSts
from cola2_safety.cfg import VirtualCageInfoConfig

from cola2_lib.rosutils import param_loader
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper

class VirtualCage(object):
    """
    This node checks if the vehicle moves beyond some given virtual limits defined in NED coordinates
    """

    def __init__(self, name):
        """ Init the class. """
        self.name = name

        # Get config parameters
        self.north_origin = -50.0
        self.east_origin = -50.0
        self.north_longitude = 100
        self.east_longitude = 100
        self.get_config()
        self.virtual_cage_enabled = True
        self.navigation_enabled = True
        self.vehicle_position = [0.0, 0.0, 0.0]

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper(self.name, "soft")

        # Publisher
        namespace = rospy.get_namespace()
        self.cage_marker_pub = rospy.Publisher(self.name + "/markers/cage", Marker, queue_size = 2)

        # Subscriber
        rospy.Subscriber(namespace + "navigator/navigation", NavSts, self.update_nav_sts)

        # Timer
        rospy.Timer(rospy.Duration(1.0), self.check_cage)

        # Create dynamic reconfigure service
        self.dynamic_reconfigure_srv = Server(VirtualCageInfoConfig, self.dynamic_reconfigure_callback)


    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {north_origin}, {east_origin}, {north_longitude}, {east_longitude},
                      {enable}""".format(**config))
        self.north_origin = config.north_origin
        self.east_origin = config.east_origin
        self.north_longitude = config.north_longitude
        self.east_longitude = config.east_longitude
        self.virtual_cage_enabled = config.enable
        return config


    def check_cage(self, event):
        """ Check if the vehicle is inside or out of the virtual cage. """
        cage_marker = Marker()
        cage_marker.header.frame_id = "/world"
        cage_marker.header.stamp = rospy.Time.now()
        cage_marker.ns = "virtual_cage"
        cage_marker.id = 10
        cage_marker.type = Marker.LINE_STRIP
        cage_marker.action = Marker.ADD
        cage_marker.pose.position.x = 0.0
        cage_marker.pose.position.y = 0.0
        cage_marker.pose.position.z = self.vehicle_position[2]
        cage_marker.pose.orientation.x = 0.0
        cage_marker.pose.orientation.y = 0.0
        cage_marker.pose.orientation.z = 0.0
        cage_marker.pose.orientation.w = 1.0
        cage_marker.scale.x = 0.2
        cage_marker.scale.y = 0.2
        cage_marker.scale.z = 0.2
        cage_marker.color.r = 0.0
        cage_marker.color.g = 1.0
        cage_marker.color.b = 0.0
        cage_marker.color.a = 1.0
        cage_marker.lifetime = rospy.Duration(2.0)
        cage_marker.points.append( Point(self.north_origin, self.east_origin, 0.0) )
        cage_marker.points.append( Point(self.north_origin + self.north_longitude, self.east_origin, 0.0) )
        cage_marker.points.append( Point(self.north_origin + self.north_longitude, self.east_origin + self.east_longitude, 0.0) )
        cage_marker.points.append( Point(self.north_origin, self.east_origin + self.east_longitude, 0.0) )
        cage_marker.points.append( Point(self.north_origin, self.east_origin, 0.0) )
        cage_marker.frame_locked = True

        if self.virtual_cage_enabled and self.navigation_enabled:
            if self.vehicle_position[0] < self.north_origin or self.vehicle_position[0] > self.north_origin + \
               self.north_longitude or self.vehicle_position[1] < self.east_origin or self.vehicle_position[1] > \
               self.east_origin + self.east_longitude:
                rospy.logwarn("%s: Vehicle out of virtual cage!", self.name)
                cage_marker.color.r = 1.0
                cage_marker.color.g = 0.0
                cage_marker.color.b = 0.0
                cage_marker.color.a = 1.0
                self.diagnostic.set_level(DiagnosticStatus.WARN, 'Vehicle out of virtual cage')
            else:
                self.diagnostic.set_level(DiagnosticStatus.OK)

            self.cage_marker_pub.publish(cage_marker)


    def update_nav_sts(self, nav):
        """ Save current navigation data. """

        self.navigation_enabled = True
        self.vehicle_position = [nav.position.north,
                                 nav.position.east,
                                 nav.position.depth]

    def get_config(self):
        """ Read parameters from ROS Param Server."""
        param_dict = {'north_origin': ('north_origin', -500.0),
                      'east_origin': ('east_origin', -500.0),
                      'north_longitude': ('north_longitude', 1000.0),
                      'east_longitude': ('east_longitude', 1000.0),
                      'enabled': ('enabled', False)}

        param_loader.get_ros_params(self, param_dict)


if __name__ == '__main__':
    try:
        rospy.init_node('virtual_cage')
        vc = VirtualCage(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
