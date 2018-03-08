#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>Used to handle requests for recovery actions coming from all nodes<@@
"""

"""
Created on Mar 25 2013
Modified 11/2015
@author: narcis palomeras
"""

import rospy
from std_srvs.srv import Empty, EmptyRequest
from cola2_msgs.srv import Goto, GotoRequest
from cola2_msgs.srv import Recovery, RecoveryRequest, RecoveryResponse
from cola2_msgs.msg import Setpoints, RecoveryAction, NavSts
from cola2_lib.rosutils import param_loader



class RecoveryActions(object):
    """ This class is able to handle recovery requests coming from all the nodes """

    def __init__(self, name):
        """ Init the class """
        # Save node name
        self.name = name

        # Get config
        self.get_config()

        namespace = rospy.get_namespace()

        # Create publisher
        self.pub_thrusters = rospy.Publisher(self.name + "/thrusters_data", Setpoints, queue_size = 2)

        self.pub_external_ra = rospy.Publisher(self.name + "/external_recovery_action",
                                               RecoveryAction, queue_size = 2)
        # Subscriber
        rospy.Subscriber(namespace + "navigator/navigation", NavSts, self.update_nav_sts, queue_size=1)

        # Init service clients
        rospy.loginfo("%s: waiting for services", self.name)

        try:
            rospy.wait_for_service(namespace + 'teleoperation/set_joystick_axes_to_velocity', 20)
            self.set_joy_to_vel_srv = rospy.ServiceProxy(namespace + 'teleopeartion/set_joystick_axes_to_velocity', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: set joystick axes to velocity service is not available!", self.name)

        self.captain_clients = True
        try:
            rospy.wait_for_service(namespace + 'pilot/disable_trajectory', 20)
            self.abort_mission_srv = rospy.ServiceProxy(namespace + 'pilot/disable_trajectory', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable trajectory service not available!", self.name)

        try:
            rospy.wait_for_service(namespace + 'pilot/disable_keep_position', 2)
            self.abort_keep_pose_srv = rospy.ServiceProxy(namespace + 'pilot/disable_keep_position', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable keep position service not available!", self.name)

        try:
            rospy.wait_for_service(namespace + 'pilot/disable_goto', 2)
            self.abort_goto_srv = rospy.ServiceProxy(namespace + 'pilot/disable_goto', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: disable goto service not available!", self.name)

        try:
            rospy.wait_for_service(namespace + 'pilot/goto', 2)
            self.goto_srv = rospy.ServiceProxy(namespace + 'pilot/goto', Goto)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("%s: goto service not available!", self.name)

        if not self.captain_clients:
            self.no_captain_clients_timer = rospy.Timer(rospy.Duration(0.4), self.no_captain_clients_message)

        try:
            rospy.wait_for_service(namespace + 'pilot/disable_thrusters', 20)
            self.abort_thrusters_srv = rospy.ServiceProxy( namespace + 'pilot/disable_thrusters', Empty)
        except rospy.exceptions.ROSException:
            self.no_disable_thrusters_service_timer = rospy.Timer(rospy.Duration(0.4),
                                                                  self.no_disable_thrusters_message)

        # Create service
        self.recovery_srv = rospy.Service(self.name +'/recover', Recovery, self.recovery_action_srv)

        # Show message
        rospy.loginfo("%s: initialized", self.name)

    def update_nav_sts(self, nav):
        """Navigation callback. It saves yaw."""
        self.last_yaw = nav.orientation.yaw

    def recovery_action_srv(self, req):
        """ Callback of recovery action service """
        rospy.loginfo('%s: received recovery action', self.name)
        who = req._connection_header['callerid']
        if who != "/safety_supervisor":
            # Timestamp might not be included if service call was from command line, repack recovery action to add it
            if req.requested_action.header.stamp.secs == 0:
                ra = RecoveryAction()
                ra.header.stamp = rospy.Time.now()
                ra.error_level = req.requested_action.error_level
                ra.error_string = req.requested_action.error_string
                self.pub_external_ra.publish(ra)
            else:
                self.pub_external_ra.publish(req.requested_action)
        #Call to handle the requested action
        self.recovery_action(req.requested_action.error_level)
        ret = RecoveryResponse()
        ret.attempted = True
        return ret


    def recovery_action(self, error):
        """ This method calls the appropiate method to handle the input code """
        if error == RecoveryAction.INFORMATIVE:
            rospy.loginfo("%s: recovery action %s: INFORMATIVE",
                          self.name, error)
            # TODO: send message through modem?
        elif error == RecoveryAction.ABORT_MISSION:
            rospy.loginfo("%s: recovery action %s: ABORT_MISSION",
                          self.name, error)
            self.abort_mission()
        elif error == RecoveryAction.ABORT_AND_SURFACE:
            rospy.loginfo("%s: recovery action %s: ABORT_AND_SURFACE",
                          self.name, error)
            self.abort_mission()
            self.surface()
        elif error == RecoveryAction.EMERGENCY_SURFACE:
            rospy.loginfo("%s: recovery action %s: EMERGENCY_SURFACE",
                          self.name, error)
            self.abort_mission()
            self.emergency_surface()
        elif error == RecoveryAction.STOP_THRUSTERS:
            rospy.loginfo("%s: recovery action %s: STOP_THRUSTERS",
                          self.name, error)
            # Disable thrusters
            try:
                self.abort_thrusters_srv(EmptyRequest())
            except rospy.exceptions.ROSException:
                rospy.logerr('%s: error disabling thrusters', self.name)
        else:
            rospy.loginfo("%s: recovery action %s: INVALID ERROR CODE",
                          self.name, error)


    def abort_mission(self):
        """ This method handles abort mission """
        rospy.loginfo("%s: abort mission", self.name)
        try:
            self.abort_mission_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the mission', self.name)

        try:
            self.abort_goto_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the goto', self.name)

        try:
            self.abort_keep_pose_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error aborting the keep pose', self.name)



    def surface(self):
        """ This method handles surface recovery action """
        rospy.loginfo("%s: surface", self.name)

        # If we are controlling with the joystick in position (Z),
        # submerge service could fail.
        # Then, we first set all joystick axes to velocity
        self.set_joy_to_vel_srv(EmptyRequest())

        try:
            goto = GotoRequest()
            goto.priority = GoalDescriptor.PRIORITY_SAFETY_HIGH
            goto.altitude = self.controlled_surface_depth
            goto.altitude_mode = False
            goto.blocking = False
            goto.keep_position = False
            goto.disable_axis.x = True
            goto.disable_axis.y = True
            goto.disable_axis.z = False
            goto.disable_axis.roll = True
            goto.disable_axis.pitch = True
            goto.disable_axis.yaw = False
            goto.position.z = self.controlled_surface_depth
            goto.position_tolerance.z = 1.0
            goto.yaw = self.last_yaw
            goto.orientation_tolerance.yaw = 0.1
            goto.reference = GotoRequest.REFERENCE_NED

            self.goto_srv(goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error surfacing the vehicle', self.name)


    def emergency_surface(self):
        """ This method handles an emergency surface """
        rospy.loginfo("%s: emergency surface", self.name)
        try:
            self.abort_thrusters_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error disabling thrusters', self.name)

        r = rospy.Rate(10)
        thrusters = Setpoints()
        while True:
            thrusters.header.stamp = rospy.Time.now()
            thrusters.setpoints = self.emergency_surface_setpoints
            self.pub_thrusters.publish(thrusters)
            r.sleep()


    def get_config(self):
        """ Get config from param server """
        param_dict = {'frame_id': ('frame_id', "girona500"),
                      'emergency_surface_setpoints': ('emergency_surface_setpoints', [0.0, 0.0, 0.75, 0.75, 0.0]),
                      'controlled_surface_depth': ('controlled_surface_depth', 0.0)}

        param_loader.get_ros_params(self, param_dict)


    def no_disable_thrusters_message(self, event):
        """ Timer to show an error in disable thrusters service """
        rospy.logfatal('%s: error creating client to disable thrusters', self.name)


    def no_captain_clients_message(self, event):
        """ Timer to show an error if unavailable captain service """
        rospy.logfatal('%s: error creating some captain clients', self.name)


if __name__ == '__main__':
    try:
        rospy.init_node('recovery_actions')
        recovery_actions = RecoveryActions(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
