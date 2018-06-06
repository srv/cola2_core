#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

from __future__ import print_function
# Basic ros
import rospy
import tf
# Import msgs
from cola2_msgs.msg import DVL  # dvl
from sensor_msgs.msg import FluidPressure  # depth
from sensor_msgs.msg import Imu  # imu
from sensor_msgs.msg import NavSatFix  # gps
from sensor_msgs.msg import Range  # altitude
from sensor_msgs.msg import Temperature  # temperature
from diagnostic_msgs.msg import DiagnosticStatus  # diagnostics
from cola2_msgs.msg import Float32Stamped  # sound velocity
from nav_msgs.msg import Odometry  # orientation of the vehicle
# Custom libraries
from cola2_lib.utils.ned import NED
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from cola2_lib.rosutils.transform_handler import TransformHandler
from cola2_lib.rosutils.param_loader import get_ros_params
# Python libraries
import numpy as np

"""
@@>Simulates the Navigation sensors of an AUV.<@@
"""


def transform_from_tf(xyz, rpy):
    """Convert a transform (xyz, rpy) of TransformHandler to numpy (xyz, rotation)."""
    v = np.array([[xyz[0], xyz[1], xyz[2]]]).T
    r = tf.transformations.euler_matrix(rpy[0], rpy[1], rpy[2])[:3, :3]
    # inverse transform
    return -r.T.dot(v), r.T


class SimAUVNavSensors(object):
    """Simulate all the sensors in Girona500."""

    def __init__(self):
        """Constructor that gets config, publishers and subscribers."""
        # Get namespace and config
        self.ns = rospy.get_namespace()
        self.get_config()
        self.simulate_altidude = True  # simulate until altitude received from simulator
        self.ned = NED(self.latitude, self.longitude, 0.0)  # NED frame

        # Set up diagnostics
        self.diagnostic_gps = DiagnosticHelper("gps", "Simulated")
        self.diagnostic_depth = DiagnosticHelper("pressure", "Simulated")
        self.diagnostic_dvl = DiagnosticHelper("dvl", "Simulated")
        self.diagnostic_imu = DiagnosticHelper("imu", "Simulated")

        # Create publishers
        self.pub_gps = rospy.Publisher(self.ns + 'navigator/gps', NavSatFix, queue_size=2)
        self.pub_depth = rospy.Publisher(self.ns + 'navigator/pressure', FluidPressure, queue_size=2)
        self.pub_dvl = rospy.Publisher(self.ns + 'navigator/dvl', DVL, queue_size=2)
        self.pub_altitude = rospy.Publisher(self.ns + 'navigator/altitude', Range, queue_size=2)
        self.pub_imu = rospy.Publisher(self.ns + 'navigator/imu', Imu, queue_size=2)
        self.pub_sound_vel = rospy.Publisher(self.ns + 'navigator/sound_velocity', Float32Stamped, queue_size=2)
        self.pub_temperature = rospy.Publisher(
            self.ns + 'valeport_sound_velocity/temperature', Temperature, queue_size=2)

        # Create subscribers
        # Odometry from dynamics to know where the vehicle is
        self.has_odom = False
        rospy.Subscriber(self.ns + 'dynamics/odometry', Odometry, self.update_odometry, queue_size=1)
        # Altitude from the simulator
        rospy.Subscriber(self.ns + 'dynamics/altitude', Range, self.update_altitude, queue_size=1)

        # Get transforms
        found = False
        self.tf_handler = TransformHandler()
        while (not found) and (not rospy.is_shutdown()):
            try:
                # GPS
                _, gps_xyz, gps_rpy = self.tf_handler.get_transform(self.ns + 'gps')
                self.tf_gps = transform_from_tf(gps_xyz, gps_rpy)
                rospy.loginfo("gps tf loaded")
                # Depth
                _, depth_xyz, depth_rpy = self.tf_handler.get_transform(self.ns + 'pressure')
                self.tf_depth = transform_from_tf(depth_xyz, depth_rpy)
                rospy.loginfo("depth tf loaded")
                # DVL
                _, dvl_xyz, dvl_rpy = self.tf_handler.get_transform(self.ns + 'dvl')
                self.tf_dvl = transform_from_tf(dvl_xyz, dvl_rpy)
                rospy.loginfo("dvl tf loaded")
                # IMU
                _, imu_xyz, imu_rpy = self.tf_handler.get_transform(self.ns + 'imu_filter')
                self.tf_imu_filter = transform_from_tf(imu_xyz, imu_rpy)
                rospy.loginfo("imu tf loaded")
                # All ok
                found = True
            except Exception as e:
                rospy.logwarn("cannot find all transforms")
                rospy.logwarn(e.message)
                rospy.sleep(2.0)

        # Init simulated sensors
        if self.gps_period > 0:
            rospy.Timer(rospy.Duration(self.gps_period), self.publish_gps)
        if self.depth_period > 0:
            rospy.Timer(rospy.Duration(self.depth_period), self.publish_depth)
        if self.dvl_period > 0:
            rospy.Timer(rospy.Duration(self.dvl_period), self.publish_dvl)
        if self.imu_period > 0:
            rospy.Timer(rospy.Duration(self.imu_period), self.publish_imu)

    def get_config(self):
        """Define and load all necessary parameters from ROS param server."""
        # Define params to load
        param_dict = {
            # absolutes
            'latitude': (self.ns + "dynamics/ned_origin_latitude", 41.0),
            'longitude': (self.ns + "dynamics/ned_origin_longitude", 3.0),
            'water_density': (self.ns + "navigator/water_density", 1030.0),
            # private
            'sea_bottom_depth': ("sea_bottom_depth", 1),
            'sound_speed': ("sound_speed", 1500.0),
            'gps_period': ("gps_period", 0.5),
            'depth_period': ("depth_period", 0.5),
            'dvl_period': ("dvl_period", 0.5),
            'imu_period': ("imu_period", 0.5),
            # cov
            'gps_position_covariance': ("gps_position_covariance", 0.25),
            'depth_pressure_covariance': ("depth_pressure_covariance", 0.01),
            'dvl_velocity_covariance': ("dvl_velocity_covariance", 0.01),
            'imu_orientation_covariance': ("imu_orientation_covariance", 0.01),
            # output cov
            'output_gps_position_covariance': ("output_gps_position_covariance", 0.5),
            'output_depth_pressure_covariance': ("output_depth_pressure_covariance", 0.1),
            'output_dvl_velocity_covariance': ("output_dvl_velocity_covariance", 0.1),
            'output_imu_orientation_covariance': ("output_imu_orientation_covariance", 0.1)}
        # Load them
        get_ros_params(self, param_dict)

    def update_odometry(self, msg):
        """Get odometry from dynamics to know where the vehicle is."""
        self.has_odom = True
        self.odom = msg
        self.rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                             msg.pose.pose.orientation.y,
                                                             msg.pose.pose.orientation.z,
                                                             msg.pose.pose.orientation.w])

    def update_altitude(self, msg):
        """If altitude is computed outside, stop computing our own."""
        self.simulate_altidude = False
        if (msg.min_range < msg.range < msg.max_range):
            self.altitude = msg.range
        else:
            self.altitude = -1.0              # invalid altitude

    def publish_gps(self, event):
        """Publish GPS according to the current position and if close to surface."""
        # Exit if no odom
        if not self.has_odom:
            rospy.loginfo("waiting for dynamics odometry")
            return
        # Compute position with noise
        north = self.odom.pose.pose.position.x + np.random.normal(0.0, self.gps_position_covariance[0])
        east = self.odom.pose.pose.position.y + np.random.normal(0.0, self.gps_position_covariance[1])
        ned = np.array([[north, east, 0.0]]).T
        # Transform to sensor
        ned = self.tf_gps[1].dot(ned) + self.tf_gps[0]
        # Transform to lat lon
        lat, lon, _ = self.ned.ned2geodetic([ned[0], ned[1], 0.0])
        # Create message
        gps = NavSatFix()
        gps.header.stamp = event.current_real
        gps.header.frame_id = self.ns + 'gps'
        gps.status.status = gps.status.STATUS_FIX
        gps.status.service = gps.status.SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = 0.0
        gps.position_covariance[0] = self.output_gps_position_covariance[0]
        gps.position_covariance[4] = self.output_gps_position_covariance[1]
        gps.position_covariance[8] = 1.0
        gps.position_covariance_type = gps.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # Good GPS data only near to surface
        if self.odom.pose.pose.position.z > 1.0:
            gps.status.status = gps.status.STATUS_NO_FIX
        # Publish
        self.pub_gps.publish(gps)
        # Diagnostic message
        self.diagnostic_gps.set_level(DiagnosticStatus.OK)

    def publish_depth(self, event):
        """Publish depth according to the current position."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        rot = tf.transformations.euler_matrix(*self.rpy)[:3, :3]
        depth_xyz = rot.dot(self.tf_depth[0])
        depth = self.odom.pose.pose.position.z - depth_xyz[2] + np.random.normal(0.0, self.depth_pressure_covariance)
        depth = max(depth, 0.01)
        pressure = depth * self.water_density * 9.81  # pascals
        # Pressure
        msg = FluidPressure()
        msg.header.stamp = event.current_real
        msg.header.frame_id = self.ns + 'pressure'
        msg.fluid_pressure = pressure
        msg.variance = self.output_depth_pressure_covariance
        self.pub_depth.publish(msg)
        # Sound velocity
        msg = Float32Stamped()
        msg.data = self.sound_speed
        self.pub_sound_vel.publish(msg)
        # Temperature
        msg = Temperature()
        msg.header.stamp = event.current_real
        msg.header.frame_id = self.ns + 'pressure'
        msg.temperature = 15.42
        self.pub_temperature.publish(msg)
        # Diagnostic message
        self.diagnostic_depth.set_level(DiagnosticStatus.OK)

    def publish_dvl(self, event):
        """Publish DVL according to the current velocity."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        u = self.odom.twist.twist.linear.x + np.random.normal(0.0, self.dvl_velocity_covariance[0])
        v = self.odom.twist.twist.linear.y + np.random.normal(0.0, self.dvl_velocity_covariance[1])
        w = self.odom.twist.twist.linear.z + np.random.normal(0.0, self.dvl_velocity_covariance[2])
        vel = np.array([u, v, w])
        # Velocity is computed at the gravity center but we want the velocity at the sensor.
        # Vdvl = V + (v.ang.z x dist(dvl->))
        ang_vel = np.array([self.odom.twist.twist.angular.x,
                            self.odom.twist.twist.angular.y, self.odom.twist.twist.angular.z])
        vel = vel + np.cross(ang_vel, self.tf_dvl[0].ravel())
        dvl = self.tf_dvl[1].dot(vel)  # rotate the dvl
        # If simulated altitude, compute it
        if self.simulate_altidude:
            self.altitude = self.sea_bottom_depth - self.odom.pose.pose.position.z
            if self.altitude < 0.5:
                self.altitude = -1.0
        # DVL
        msg = DVL()
        msg.header.stamp = event.current_real
        msg.header.frame_id = self.ns + 'dvl'
        msg.velocity.x = dvl[0]
        msg.velocity.y = dvl[1]
        msg.velocity.z = dvl[2]
        msg.velocity_covariance[0] = self.output_dvl_velocity_covariance[0]
        msg.velocity_covariance[4] = self.output_dvl_velocity_covariance[1]
        msg.velocity_covariance[8] = self.output_dvl_velocity_covariance[2]
        msg.altitude = self.altitude
        self.pub_dvl.publish(msg)
        # Altitude
        msg = Range()
        msg.header.stamp = event.current_real
        msg.header.frame_id = self.ns + 'dvl_altitude'
        msg.radiation_type = msg.ULTRASOUND
        msg.field_of_view = 0.2
        msg.min_range = 0.5
        msg.max_range = 80.0
        msg.range = self.altitude
        self.pub_altitude.publish(msg)
        # Diagnostic message
        self.diagnostic_dvl.set_level(DiagnosticStatus.OK)

    def publish_imu(self, event):
        """Publish IMU according to the current orientation."""
        # Exit if no odom
        if not self.has_odom:
            return
        # Measurement
        r = self.rpy[0] + np.random.normal(0.0, self.imu_orientation_covariance[0])
        p = self.rpy[1] + np.random.normal(0.0, self.imu_orientation_covariance[1])
        y = self.rpy[2] + np.random.normal(0.0, self.imu_orientation_covariance[2])
        rot = tf.transformations.euler_matrix(r, p, y)[:3, :3]
        # TODO: Is this right? Maybe vehicle_rpy * self.imu_tf.M instead???
        wrot = np.eye(4)
        wrot[:3, :3] = self.tf_imu_filter[1].dot(rot)
        quat = tf.transformations.quaternion_from_matrix(wrot)
        # IMU
        msg = Imu()
        msg.header.stamp = event.current_real
        msg.header.frame_id = self.ns + 'imu_filter'
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.orientation_covariance[0] = self.output_imu_orientation_covariance[0]
        msg.orientation_covariance[4] = self.output_imu_orientation_covariance[1]
        msg.orientation_covariance[8] = self.output_imu_orientation_covariance[2]
        # TODO: WARNING! the angular velocity is not rotated!
        msg.angular_velocity = self.odom.twist.twist.angular
        msg.angular_velocity_covariance[0] = 0.1
        msg.angular_velocity_covariance[4] = 0.1
        msg.angular_velocity_covariance[8] = 0.1
        self.pub_imu.publish(msg)
        # Diagnostic message
        self.diagnostic_imu.set_level(DiagnosticStatus.OK)


if __name__ == '__main__':
    # init
    rospy.init_node('sim_auv_nav_sensors')
    node = SimAUVNavSensors()
    rospy.spin()
