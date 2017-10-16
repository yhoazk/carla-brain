#!/usr/bin/python
"""
Test cases
"""
import sys
import time
import threading
import unittest
import rospy
import rostest
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd

PKG = 'twist_controller'
NAME = 'test_twist_controller'


class TestTwistController(unittest.TestCase):
    """
    A basic testcase for the twist controller.
    It assumes that the twist controller will send in a constant rate
    the control information.
    When dbw_enabed is set to False, no control information shall be send
    anymore as of user interaction.
    """

    def __init__(self, *args):
        super(TestTwistController, self).__init__(*args)
        self.success = False
        self.failed_once = False
        self.dbw_enabled = True
        self.dbw_enabled_callback = self.dbw_enabled
        self.lock = threading.Lock()

    def callback(self, data):
        """
        Right now we're happy to get a lane
        :param data: The callback information
        """
        with self.lock:
            value = self.dbw_enabled_callback
            if value:
                self.success = False if self.failed_once else True
            elif not self.dbw_enabled: # Last check if we already switched back.
                self.failed_once = True
                self.success = False

        rospy.loginfo("%sxpected callback. dbw_enabled_callback %r, dbw_enabled %r, Failed before %r",
                      "E" if value else "Une", value, self.dbw_enabled, self.failed_once)


    def test_notify(self):
        """
        Test to receive a waypoint.
        To trigger this, images needs to be published.
        """
        rospy.init_node(NAME, anonymous=True)
        basepub = rospy.Publisher('/vehicle/dbw_enabled', Bool)
        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.callback)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.callback)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.callback)
        count = 1
        while not rospy.is_shutdown() and count < 10:
            timeout_t = time.time() + 1.0  # 1 second
            self.dbw_enabled = False if self.dbw_enabled else True
            """
            Immediately react if dbw is enabled
            """
            while time.time() < timeout_t:
                basepub.publish(Bool(self.dbw_enabled))
                rospy.logwarn("dbw_enabled: %r", self.dbw_enabled)
                time.sleep(0.01)
                """
                Grace period if switching to dbw disabled
                """
                with self.lock:
                    self.dbw_enabled_callback = self.dbw_enabled
            count += 1
        self.assert_(self.success, str(self.success))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTwistController, sys.argv)
