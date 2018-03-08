#!/usr/bin/env python
# Copyright (c) 2018 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

class ErrorCode:
    """
    Class to decodify the error_code represented in 16bits (2 bytes)
    into different vehicle errors and warnings.
    """

    """
    Definitions of each bit number (will be 1 if the error or warning is present)
    """
    WATCHDOG_TIMER = 15
    BAT_WARNING = 14
    BAT_ERROR = 13
    NAV_STS_WARNING = 12
    NAV_STS_ERROR = 11
    INTERNAL_SENSORS_WARNING = 10
    INTERNAL_SENSORS_ERROR = 9
    DVL_BOTTOM_FAIL = 8
    CURRENT_WAYPOINT_BASE = 7 # to 0

    def __init__(self):
        pass

    def unpack(self, error_code):
        """
        Unpacks an error_code string to a dictionary of errors and warnings
        """
        error_code_string = format(error_code, '016b')

        watchdog_timer = (error_code_string[ErrorCode.WATCHDOG_TIMER] == '1')
        bat_warning = (error_code_string[ErrorCode.BAT_WARNING] == '1')
        bat_error = (error_code_string[ErrorCode.BAT_ERROR] == '1')
        nav_sts_warning = (error_code_string[ErrorCode.NAV_STS_WARNING] == '1')
        nav_sts_error = (error_code_string[ErrorCode.NAV_STS_ERROR] == '1')
        internal_sensors_warning = (error_code_string[ErrorCode.INTERNAL_SENSORS_WARNING] == '1')
        internal_sensors_error = (error_code_string[ErrorCode.INTERNAL_SENSORS_ERROR] == '1')
        dvl_bottom_fail = (error_code_string[ErrorCode.DVL_BOTTOM_FAIL] == '1')
        current_waypoint = int(error_code_string[0:ErrorCode.CURRENT_WAYPOINT_BASE + 1], 2)

        return {'watchdog_timer': watchdog_timer,
        		'bat_warning': bat_warning,
        		'bat_error': bat_error,
        		'nav_sts_warning': nav_sts_warning,
        		'nav_sts_error': nav_sts_error,
        		'internal_sensors_warning': internal_sensors_warning,
        		'internal_sensors_error': internal_sensors_error,
        		'dvl_bottom_fail': dvl_bottom_fail,
        		'current_waypoint': current_waypoint}
