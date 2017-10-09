#!/usr/bin/python
"""
Test cases
"""
import sys
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

PKG = 'waypoint_updater'
NAME = 'test_waypoint_updater'

class TestWaypointUpdater(unittest.TestCase):
    """
    A basic testcase for the waypoint_updater
    """
    def __init__(self, *args):
        super(TestWaypointUpdater, self).__init__(*args)
        self.success = False

    def callback(self, data):
        """
        Right now we're happy to get a lane
        :param data: The callback information
        """
        rospy.logwarn(data)
        self.success = True

    def test_notify(self):
        """
        Test to receive a waypoint.
        To trigger this, images needs to be published.
        """
        rospy.init_node(NAME, anonymous=True)
        posepub = rospy.Publisher('/current_pose', PoseStamped)
        basepub = rospy.Publisher('/base_waypoints', Lane)
        rospy.Subscriber("final_waypoints", Lane, self.callback)
        timeout_t = time.time() + 100.0 * 1000  # 10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            posepub.publish(self._get_posestamped())
            basepub.publish(self._get_lane())
            #pub.publish(None, None, None, None, None, None, None)
            #rospy.logwarn("published image")
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))

    def _get_lane(self):
        lane = Lane()
        lane.header.frame_id = "/base_waypoints"
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = [self._get_waypoint(-1), self._get_waypoint(0),
                          self._get_waypoint(1), self._get_waypoint(2)]
        return lane

    def _get_waypoint(self, value=0.0):
        waypoint = Waypoint()
        waypoint.pose = self._get_posestamped(value)
        waypoint.twist = self._get_twiststamped(0.0)
        return waypoint

    @classmethod
    def _get_posestamped(cls, value=0.0):
        goal = PoseStamped()
        goal.header.frame_id = "/current_pose"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = value
        goal.pose.position.x = value
        goal.pose.position.y = value
        goal.pose.orientation.w = 1.0
        return goal

    @classmethod
    def _get_twiststamped(cls, value=0.0):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "/current_pose"
        twist.twist.linear.x = value
        twist.twist.linear.y = value
        twist.twist.linear.z = value
        twist.twist.angular.x = value
        twist.twist.angular.y = value
        twist.twist.angular.z = value
        return twist


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointUpdater, sys.argv)
