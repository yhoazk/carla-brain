#!/usr/bin/python
"""
Test cases
"""
import sys
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

PKG = 'tl_detector'
NAME = 'test_tl_detector'

class TestTlDetector(unittest.TestCase):
    """
    A basic testcase for the tl_detector
    """
    def __init__(self, *args):
        super(TestTlDetector, self).__init__(*args)
        self.success = False

    def callback(self, data):
        """
        -1 is only sent over peer_publish callback, so hearing it is a success condition
        :param data: The callback information
        """
        rospy.logwarn("I heard %d", data.data)
        if data.data == -1:
            self.success = True

    def test_notify(self):
        """
        Test to receive a waypoint.
        To trigger this, images needs to be published.
        """
        pub = rospy.Publisher('/image_color', Image)
        rospy.Subscriber("traffic_waypoint", Int32, self.callback)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 10.0 * 1000  # 10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            pub.publish(None, None, None, None, None, None, None)
            rospy.logwarn("published image")
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTlDetector, sys.argv)
