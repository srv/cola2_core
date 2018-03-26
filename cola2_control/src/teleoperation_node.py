#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
@@>This nodes takes the data coming from a joystick device and generates a World Waypoint Request
or a Body Velocity Request. <@@
"""

# ROS imports
import rospy

# Import messages
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticStatus
from cola2_msgs.msg import BodyVelocityReq
from cola2_msgs.msg import WorldWaypointReq
from cola2_msgs.msg import GoalDescriptor
from cola2_msgs.msg import NavSts
from cola2_msgs.srv import MaxJoyVelocity, MaxJoyVelocityResponse
from cola2_lib.rosutils.diagnostic_helper import DiagnosticHelper
from cola2_lib.rosutils import param_loader
from cola2_lib.utils import angles
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class Teleoperation(object):
    """ This class recieves a joy message and generates a world_waypoint_req
        or a body_velocity_req.

        The joy message always have the same structure. The axis contain
        the value for pose and twist:
        --> axis: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
        While the buttons decide if an axis is controlled in pose or in twist:
        --> buttons: [x][y][z][roll][pitch][yaw][u][v][w][p][q][r]
    """

    def __init__(self, name):
        """ Constructor """
        self.name = name
        self.last_map_ack = 0.0

        namespace = rospy.get_namespace()

        # Set up diagnostics
        self.diagnostic = DiagnosticHelper('teleoperation', "soft")

        # Some vars
        self.map_ack_init = False
        self.map_ack_alive = True
        self.manual_pitch = False
        self.seq = 0
        self.nav_init = False
        self.base_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.nav_init = False

        # Get config
        self.actualize_base_pose = True  # Default
        self.get_config()

        # Create publishers
        self.pub_body_velocity_req = rospy.Publisher(
            namespace+'controller/body_velocity_req',
            BodyVelocityReq,
            queue_size=2)

        self.pub_world_waypoint_req = rospy.Publisher(
            namespace+'controller/world_waypoint_req',
            WorldWaypointReq,
            queue_size=2)

        self.pub_check_joystick = rospy.Publisher(
            self.name+'/ack',
            String,
            queue_size=2)

        # Create subscribers
        rospy.Subscriber(namespace+"input_to_teleoperation/ack",
                         String,
                         self.ack_callback,
                         queue_size=1)

        rospy.Subscriber(namespace+"input_to_teleoperation/output",
                         Joy,
                         self.output_callback,
                         queue_size=1)

        rospy.Subscriber(namespace+"navigator/navigation",
                         NavSts,
                         self.nav_sts_update,
                         queue_size=1)

        # Create services
        self.set_joy_srv = rospy.Service(
            self.name+'/set_max_joy_velocity',
            MaxJoyVelocity,
            self.set_max_joy_vel)

        self.set_axes_velocity_srv = rospy.Service(
            self.name+'/set_joystick_axes_to_velocity',
            Empty,
            self.set_axes_velocity)

        self.reload_config_srv = rospy.Service(
            self.name+'/reload_joystick_config',
            Empty,
            self.reload_joystick_config_srv)

        # Init periodic check timer
        rospy.Timer(rospy.Duration(1), self.check_map_ack)

    def nav_sts_update(self, data):
        """ This is the callback for the navigation message """
        self.nav_init = True
        self.last_pose[0] = data.position.north
        self.last_pose[1] = data.position.east
        self.last_pose[2] = data.position.depth
        self.last_pose[3] = data.orientation.roll
        self.last_pose[4] = data.orientation.pitch
        self.last_pose[5] = data.orientation.yaw

    def ack_callback(self, ack_msg):
        """ This is the callback for the ack safety message """
        data = ack_msg.data.split(' ')
        if data[1] == 'ack' and data[0] == str(self.seq + 1):
            self.map_ack_alive = True
            self.map_ack_init = True
            self.seq = self.seq + 1
            self.last_map_ack = rospy.Time.now().to_sec()

    def check_map_ack(self, event):
        """ This is a callback for a timer. It publishes ack safety message
            and pose and velocity safety messages if map_ack is lost """
        if self.map_ack_init:
            self.diagnostic.add(
                "last_ack",
                str(rospy.Time.now().to_sec() - self.last_map_ack))
            if self.map_ack_alive:
                self.map_ack_alive = False
                self.diagnostic.set_level(DiagnosticStatus.OK)
            else:
                rospy.loginfo("%s: we have lost map_ack!", self.name)
                self.diagnostic.set_level(
                    DiagnosticStatus.WARN,
                    'Communication with map_ack lost!')
                body_velocity_req = BodyVelocityReq()
                body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION_LOW
                body_velocity_req.goal.requester = self.name + '_vel'
                body_velocity_req.twist.linear.x = 0.0
                body_velocity_req.twist.linear.y = 0.0
                body_velocity_req.twist.linear.z = 0.0
                body_velocity_req.twist.angular.x = 0.0
                body_velocity_req.twist.angular.y = 0.0
                body_velocity_req.twist.angular.z = 0.0
                body_velocity_req.disable_axis.x = True
                body_velocity_req.disable_axis.y = True
                body_velocity_req.disable_axis.z = True
                body_velocity_req.disable_axis.roll = True
                body_velocity_req.disable_axis.pitch = True
                body_velocity_req.disable_axis.yaw = True
                body_velocity_req.header.stamp = rospy.Time().now()
                body_velocity_req.header.frame_id = rospy.get_namespace() + "/base_link"
                self.pub_body_velocity_req.publish(body_velocity_req)

                world_waypoint_req = WorldWaypointReq()
                world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION_LOW
                world_waypoint_req.goal.requester = self.name + '_pose'
                world_waypoint_req.disable_axis.x = True
                world_waypoint_req.disable_axis.y = True
                world_waypoint_req.disable_axis.z = True
                world_waypoint_req.disable_axis.roll = True
                world_waypoint_req.disable_axis.pitch = True
                world_waypoint_req.disable_axis.yaw = True
                world_waypoint_req.header.stamp = rospy.Time().now()
                world_waypoint_req.header.frame_id = "/ned"
                self.pub_world_waypoint_req.publish(world_waypoint_req)
        else:
            rospy.loginfo("%s: waiting for map ack...", self.name)

        # Send ack message
        msg = String()
        msg.data = str(self.seq) + ' ok'
        self.pub_check_joystick.publish(msg)

    def output_callback(self, data):
        """ This is the main callback. Data is recieved, processed and sent
            to pose and velocity controllers """

        # Compute desired positions and velocities
        desired = [0 for x in range(12)]
        for i in range(6):
            if data.axes[i] < 0:
                desired[i] = abs(data.axes[i]) * self.min_pos[i] + self.base_pose[i]
            else:
                desired[i] = data.axes[i] * self.max_pos[i] + self.base_pose[i]
            if i > 2:
                # Normalize angles
                desired[i] = angles.wrap_angle(desired[i])

        for i in range(6, 12):
            if data.axes[i] < 0:
                desired[i] = abs(data.axes[i]) * self.min_vel[i - 6]
            else:
                desired[i] = data.axes[i] * self.max_vel[i - 6]

        # Check if pose controller is enabled
        for b in range(6):
            if data.buttons[b] == 1:
                self.pose_controlled_axis[b] = True
                if self.actualize_base_pose:
                    self.base_pose[b] = self.last_pose[b]
                rospy.loginfo("%s: axis %s now is pose", self.name, str(b))

        # Check if velocity controller is enabled
        for b in range(6, 12):
            if data.buttons[b] == 1:
                self.pose_controlled_axis[b - 6] = False
                rospy.loginfo("%s: axis %s now is velocity", self.name, str(b - 6))

        if self.nav_init:
            # Positions
            world_waypoint_req = WorldWaypointReq()
            world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION
            world_waypoint_req.goal.requester = self.name + '_pose'
            world_waypoint_req.position.north = desired[0]
            world_waypoint_req.position.east = desired[1]
            world_waypoint_req.position.depth = desired[2]
            world_waypoint_req.orientation.roll = desired[3]
            world_waypoint_req.orientation.pitch = desired[4]
            world_waypoint_req.orientation.yaw = desired[5]
            world_waypoint_req.disable_axis.x = not self.pose_controlled_axis[0]
            world_waypoint_req.disable_axis.y = not self.pose_controlled_axis[1]
            world_waypoint_req.disable_axis.z = not self.pose_controlled_axis[2]
            world_waypoint_req.disable_axis.roll = not self.pose_controlled_axis[3]
            world_waypoint_req.disable_axis.pitch = not self.pose_controlled_axis[4]
            world_waypoint_req.disable_axis.yaw = not self.pose_controlled_axis[5]
            world_waypoint_req.header.stamp = rospy.Time().now()
            world_waypoint_req.header.frame_id = "/ned"

            # if not world_waypoint_req.disable_axis.pitch:
            #    rospy.logfatal("%s: PITCH IS NOT DISABLED!", self.name)
            #    world_waypoint_req.disable_axis.pitch = True

            if (world_waypoint_req.disable_axis.x and
                    world_waypoint_req.disable_axis.y and
                    world_waypoint_req.disable_axis.z and
                    world_waypoint_req.disable_axis.roll and
                    world_waypoint_req.disable_axis.pitch and
                    world_waypoint_req.disable_axis.yaw):
                world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION_LOW

            self.pub_world_waypoint_req.publish(world_waypoint_req)

            # Velocities
            body_velocity_req = BodyVelocityReq()
            body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION
            body_velocity_req.goal.requester = self.name + '_vel'
            body_velocity_req.twist.linear.x = desired[6]
            body_velocity_req.twist.linear.y = desired[7]
            body_velocity_req.twist.linear.z = desired[8]
            body_velocity_req.twist.angular.x = desired[9]
            body_velocity_req.twist.angular.y = desired[10]
            body_velocity_req.twist.angular.z = desired[11]
            body_velocity_req.disable_axis.x = self.pose_controlled_axis[0]
            body_velocity_req.disable_axis.y = self.pose_controlled_axis[1]
            body_velocity_req.disable_axis.z = self.pose_controlled_axis[2]
            body_velocity_req.disable_axis.roll = self.pose_controlled_axis[3]
            body_velocity_req.disable_axis.pitch = self.pose_controlled_axis[4]
            body_velocity_req.disable_axis.yaw = self.pose_controlled_axis[5]

            # Check if DoF is disable
            if abs(body_velocity_req.twist.linear.x) < 0.025:
                body_velocity_req.disable_axis.x = True

            if abs(body_velocity_req.twist.linear.y) < 0.025:
                body_velocity_req.disable_axis.y = True

            if abs(body_velocity_req.twist.linear.z) < 0.025:
                body_velocity_req.disable_axis.z = True

            if abs(body_velocity_req.twist.angular.x) < 0.01:
                body_velocity_req.disable_axis.roll = True

            if abs(body_velocity_req.twist.angular.y) < 0.01:
                body_velocity_req.disable_axis.pitch = True

            if abs(body_velocity_req.twist.angular.z) < 0.01:
                body_velocity_req.disable_axis.yaw = True

            # If all DoF are disabled set priority to LOW
            if (body_velocity_req.disable_axis.x and
                    body_velocity_req.disable_axis.y and
                    body_velocity_req.disable_axis.z and
                    body_velocity_req.disable_axis.roll and
                    body_velocity_req.disable_axis.pitch and
                    body_velocity_req.disable_axis.yaw):
                body_velocity_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION_LOW

            # Publish message
            body_velocity_req.header.stamp = rospy.Time().now()
            body_velocity_req.header.frame_id = rospy.get_namespace() + "/base_link"
            self.pub_body_velocity_req.publish(body_velocity_req)

    def get_config(self):
        """ Get config from param server """
        param_dict = {'max_pos': ('max_pos',
                                  [0.0, 0.0, 0.0, 3.14159265359, 1.0, 3.14159265359]),
                      'min_pos': ('min_pos',
                                  [0.0, 0.0, -2.0, -3.14159265359, -1.0, -3.14159265359]),
                      'max_vel': ('max_vel',
                                  [0.1, 0.0, 0.1, 0.0, 0.0, 0.1]),
                      'min_vel': ('min_vel',
                                  [-0.1, 0.0, -0.1, 0.0, 0.0, -0.1]),
                      'pose_controlled_axis': ('pose_controlled_axis',
                                               [False, False, False, False, False, False]),
                      'base_pose': ('base_pose',
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                      'actualize_base_pose': ('actualize_base_pose',
                                              True)}

        param_loader.get_ros_params(self, param_dict)



    def set_max_joy_vel(self, req):
        """ Change max/min joy velocity."""
        rospy.loginfo("%s: change max/min joy velocity", self.name)
        for i in range(6):
            self.max_vel[i] = req.max_joy_velocity[i]
            self.min_vel[i] = -req.max_joy_velocity[i]

        return MaxJoyVelocityResponse(True)

    def set_axes_velocity(self, req):
        """ Set all joystick axes to velocity control"""
        data = Joy()
        rospy.loginfo("%s: Set all axis to velocity", self.name)
        # Set all axis at 0.0 and set control to velocity for all axes
        for i in range(12):
            data.axes.append(0.0)
            if i < 6:
                data.buttons.append(0)
            else:
                data.buttons.append(1)

        self.map_ack_data_callback(data)

        return EmptyResponse()

    def reload_joystick_config_srv(self, req):
        """ Reload joystick config values."""
        rospy.loginfo("%s: Reload teleopertion configurations.", self.name)
        self.get_config()
        return EmptyResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('teleoperation')
        teleoperation = Teleoperation(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
