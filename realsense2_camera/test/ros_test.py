#!/usr/bin/env python3

# System imports
import os
from datetime import datetime
import time
import sys
import logging

# ROS imports
import rospy
import rosnode
from any_realsense2_camera.msg import TimestampingInfoMsg
from anymal_longterm_tests import TopicTimeoutTester
from anymal_longterm_tests import TestNode

# Timing margin that a message is allowed to have before being considered
# timed out. That means if two subsequent messages have a time delta of more
# than  TOPIC_RATE_MARGIN * NOMINAL_RATE, the message is timed out. The nominal
# rate is queried from the param server. TOPIC_RATE_MARGIN should be >= 1 to
# be meaningfuk
TOPIC_RATE_MARGIN = 2.0 
# Default timeout If the nominal rate is not found on the param server
TOPIC_TIMEOUT = 0.5 
TOPIC_HARD_TIMEOUT = 30
TEST_NAME = "realsense_test"

EXPOSURE_TIME_UPPER_THRESHOLD_MSEC = 1000000


class RealsenseTester(TopicTimeoutTester):
    """
    Realsense test class that inherits from the topic timeout tester. 
    The class tests if the realsense topic times out. It keeps track
    of the number of messages and timeouts and logs the timeout length.

    Args:
        TopicTimeoutTester (TopicTimeoutTester): Base class
    """
    def __init__(self, suffix="front", timeout=TOPIC_TIMEOUT):
        """[summary]

        Args:
            suffix (str, optional): Realsense suffix. Defaults to "front".
        """
        super().__init__(topic="/depth_camera_" + suffix +
                               "/camera_timestamping_info",
                         message_type=TimestampingInfoMsg,
                         timeout=timeout,
                         hard_timeout=TOPIC_HARD_TIMEOUT,
                         custom_rx_hook=self.__is_data_in_bounds)
        self.name = "realsense_test_" + suffix
        self.frame_counter_last = None # Initialize to none

    def __is_data_in_bounds(self, d):
        # Frame counter check
        if self.frame_counter_last is not None:
            # Frame counter should always be incremented by 1.
            expected_counter = self.frame_counter_last + 1
            if d.frame_metadata.frame_counter != expected_counter:
                self.data_out_of_bounds += 1
                rospy.logerr("{}: Expected frame counter {} - received {}".format(self.name,
                    expected_counter, d.frame_metadata.frame_counter))
        # Update frame last frame counter with the received one
        self.frame_counter_last = d.frame_metadata.frame_counter
        
        # Check actual FPS
        if d.frame_metadata.actual_fps != 6.0:
            self.data_out_of_bounds += 1
            rospy.logerr("{}: Actual FPS is out of bounds in frame metadata".format(self.name))

        # Exposure time check 
        if d.frame_metadata.exposure_time < 0 or d.frame_metadata.exposure_time > EXPOSURE_TIME_UPPER_THRESHOLD_MSEC:
            self.data_out_of_bounds += 1
            rospy.logerr("{}: Exposure time is out of bounds ({} - should be below {})".format(self.name, d.frame_metadata.exposure_time, EXPOSURE_TIME_UPPER_THRESHOLD_MSEC))

        # Check if laser is enabled
        if not d.frame_metadata.laser_enabled:
            self.data_out_of_bounds += 1
            rospy.logerr("{}: Laser is reports that it is not enabled".format(self.name))    

        # Temperature (0.0) and laser power (150) never change, so they are not really meaningful to test

    def get_name(self):
        """
        Name of the tester instance

        Returns:
            str: name
        """
        return self.name


def get_enabled_realsenses():
    """
    Get the enabled realsenses from the parameter server
    and append them to RealsenseTester
    """
    realsenses = []
    if rospy.get_param('/realsense_test/front_enabled', True):
        fps = rospy.get_param('/depth_camera_front/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("front", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/rear_enabled', True):
        fps = rospy.get_param('/depth_camera_rear/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("rear", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/left_enabled', True):
        fps = rospy.get_param('/depth_camera_left/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("left", TOPIC_RATE_MARGIN / fps))
    if rospy.get_param('/realsense_test/right_enabled', True):
        fps = rospy.get_param('/depth_camera_right/any_realsense2_camera/depth_fps', default=TOPIC_TIMEOUT)
        realsenses.append(RealsenseTester("right", TOPIC_RATE_MARGIN / fps))
    return realsenses


def main():
    # Run the test and create a log when done
    rospy.init_node(TEST_NAME)
    realsense_testers = get_enabled_realsenses()
    test = TestNode(TEST_NAME, realsense_testers)
    test.run_test()


if __name__ == '__main__':
    main()
