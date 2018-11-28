#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
@@>This node is used to handle requests for recovery actions coming from all nodes.<@@
"""

import rospy
from std_srvs.srv import Empty, EmptyRequest
from std_srvs.srv import Trigger, TriggerRequest
from cola2_msgs.srv import Goto
from cola2_msgs.srv import Recovery, RecoveryResponse
from cola2_msgs.msg import Setpoints, RecoveryAction
from cola2_lib.rosutils import param_loader


class RecoveryActions(object):
    """ This class is able to handle recovery requests coming from all the nodes """

    def __init__(self):
        """ Init the class """
        # Get config
        self.get_config()

        ns = rospy.get_namespace()

        # Create publisher
        self.pub_thrusters = rospy.Publisher(ns + "controller/thruster_setpoints", Setpoints, queue_size = 2)
        self.pub_external_ra = rospy.Publisher(rospy.get_name() + "/external_recovery_action",
                                               RecoveryAction, queue_size = 2)

        # Init service clients
        rospy.loginfo("Waiting for services")

        try:
            rospy.wait_for_service(ns + 'teleoperation/set_joystick_axes_to_velocity', 20)
            self.set_joy_to_vel_srv = rospy.ServiceProxy(ns + 'teleoperation/set_joystick_axes_to_velocity', Empty)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Set joystick axes to velocity service is not available!")

        self.captain_clients = True
        try:
            rospy.wait_for_service(ns + 'captain/disable_external_mission', 20)
            self.abort_external_mission_srv = rospy.ServiceProxy(ns + 'captain/disable_external_mission', Trigger)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Disable external mission service not available!")

        try:
            rospy.wait_for_service(ns + 'captain/disable_mission', 20)
            self.abort_mission_srv = rospy.ServiceProxy(ns + 'captain/disable_mission', Trigger)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Disable mission service not available!")

        try:
            rospy.wait_for_service(ns + 'captain/disable_keep_position', 2)
            self.abort_keep_position_srv = rospy.ServiceProxy(ns + 'captain/disable_keep_position', Trigger)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Disable keep position service not available!")

        try:
            rospy.wait_for_service(ns + 'captain/disable_goto', 2)
            self.abort_goto_srv = rospy.ServiceProxy(ns + 'captain/disable_goto', Trigger)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Disable goto service not available!")

        try:
            rospy.wait_for_service(ns + 'captain/enable_goto', 2)
            self.goto_srv = rospy.ServiceProxy(ns + 'captain/enable_goto', Goto)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Goto service not available!")

        try:
            rospy.wait_for_service(ns + 'captain/enable_keep_position_non_holonomic', 2)  # Should work for both robots
            self.keep_position_srv = rospy.ServiceProxy(ns + 'captain/enable_keep_position_non_holonomic', Trigger)
        except rospy.exceptions.ROSException:
            self.captain_clients = False
            rospy.logfatal("Keep position service not available!")

        if not self.captain_clients:
            self.no_captain_clients_timer = rospy.Timer(rospy.Duration(0.4), self.no_captain_clients_message)

        try:
            rospy.wait_for_service(ns + 'controller/disable_thrusters', 20)
            self.disable_thrusters_srv = rospy.ServiceProxy(ns + 'controller/disable_thrusters', Empty)
        except rospy.exceptions.ROSException:
            self.no_disable_thrusters_service_timer = rospy.Timer(rospy.Duration(0.4),
                                                                  self.no_disable_thrusters_message)

        # Create service
        self.recovery_srv = rospy.Service(rospy.get_name() + '/recover', Recovery, self.recovery_action_srv)

        # Show message
        rospy.loginfo("Initialized")


    def recovery_action_srv(self, req):
        """ Callback of recovery action service """
        rospy.loginfo("Received recovery action")
        who = req._connection_header['callerid']
        if not "/safety_supervisor" in who:
            rospy.loginfo("Recovery action requested by an external agent")
            # Timestamp might not be included if service call was from command line. Repack recovery action to add it
            if req.requested_action.header.stamp.secs == 0:
                ra = RecoveryAction()
                ra.header.stamp = rospy.Time.now()
                ra.error_level = req.requested_action.error_level
                ra.error_string = req.requested_action.error_string
                self.pub_external_ra.publish(ra)
            else:
                self.pub_external_ra.publish(req.requested_action)
        # Call to handle the requested action
        self.recovery_action(req.requested_action.error_level)
        ret = RecoveryResponse()
        ret.attempted = True
        return ret


    def recovery_action(self, error):
        """ This method calls the appropiate method to handle the input code """
        # TODO: send message through modem?
        if error == RecoveryAction.INFORMATIVE:
            rospy.loginfo("Recovery action %s: INFORMATIVE", error)
        elif error == RecoveryAction.ABORT_MISSION:
            rospy.loginfo("Recovery action %s: ABORT_MISSION", error)
            self.abort_mission_and_goto()
        elif error == RecoveryAction.ABORT_AND_SURFACE:
            rospy.loginfo("Recovery action %s: ABORT_AND_SURFACE", error)
            self.abort_mission_and_goto()
            self.keep_position()
        elif error == RecoveryAction.EMERGENCY_SURFACE:
            rospy.loginfo("Recovery action %s: EMERGENCY_SURFACE", error)
            self.abort_mission_and_goto()
            self.abort_keep_position()
            self.disable_thrusters()
            self.emergency_surface()
        elif error == RecoveryAction.STOP_THRUSTERS:
            rospy.loginfo("Recovery action %s: STOP_THRUSTERS", error)
            self.disable_thrusters()
        else:
            rospy.loginfo("Recovery action %s: INVALID ERROR CODE", error)


    def abort_mission_and_goto(self):
        """ This method handles abort mission and goto"""
        rospy.loginfo("Abort mission and goto")
        try:
            self.abort_external_mission_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error aborting the external mission")

        try:
            self.abort_mission_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error aborting the mission")

        try:
            self.abort_goto_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error aborting the goto")


    def abort_keep_position(self):
        """ This method aborts keep position """
        rospy.loginfo("Abort keep position")
        try:
            self.abort_keep_position_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error aborting keep position")


    def keep_position(self):
        """ This method enables keep position """
        rospy.loginfo("Enabling keep position")
        try:
            self.keep_position_srv(TriggerRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error keeping position")


    def disable_thrusters(self):
        """ This method disables the thrusters """
        rospy.loginfo("Disabling thrusters")
        try:
            self.disable_thrusters_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr("Error disabling thrusters")


    def emergency_surface(self):
        """ This method handles an emergency surface """
        rospy.loginfo("Emergency surface")
        r = rospy.Rate(10)
        thrusters = Setpoints()
        thrusters.setpoints = self.emergency_surface_setpoints
        while not rospy.is_shutdown():
            thrusters.header.stamp = rospy.Time.now()
            self.pub_thrusters.publish(thrusters)
            r.sleep()


    def get_config(self):
        """ Get config from param server """
        # TODO: this is dangerous. The default emergency setpoint should be always loaded from param server instead of
        # silently taking a default value
        param_dict = {'frame_id': ('frame_id', "girona500"),
                      'emergency_surface_setpoints': ('emergency_surface_setpoints', [0.0, 0.0, 0.75, 0.75, 0.0])}
        param_loader.get_ros_params(self, param_dict)


    def no_disable_thrusters_message(self, event):
        """ Timer to show an error in disable thrusters service """
        rospy.logfatal("Error creating client to disable thrusters")


    def no_captain_clients_message(self, event):
        """ Timer to show an error if unavailable captain service """
        rospy.logfatal("Error creating some captain clients")


if __name__ == '__main__':
    try:
        rospy.init_node('recovery_actions')
        recovery_actions = RecoveryActions()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
