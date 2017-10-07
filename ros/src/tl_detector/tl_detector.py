#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np

STATE_COUNT_THRESHOLD = 3
INDEX_DISTANCE_THRESHOLD = 150


class TLDetector(object):
    """
    """
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.lights_position = np.array([])
        self.waypoints_np = np.array([])
        self.nearest_waypoints_to_lines = np.array([])
        self.car_direction = 1
        self.car_last_wp = None
        self.in_range = False
        self.last_in_range = False
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map
        space and helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available.
        You'll need to rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        """ Publish the index of the waypoint nearest to the upcoming red traffic light
        """
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        print "init done"
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        for point in waypoints.waypoints:
            x_coord = point.pose.pose.position.x
            y_coord = point.pose.pose.position.y
            self.waypoints_np = np.append(self.waypoints_np, complex(x_coord, y_coord))

        rospy.logwarn("Waypoints updated to "+ str(len(self.waypoints_np))+ " elements")

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if len(msg.lights) != len(self.lights_position):
            for position in msg.lights:
                x = position.pose.pose.position.x
                y = position.pose.pose.position.y
                self.lights_position = np.append(self.lights_position, complex(x, y))

            rospy.logwarn("tl_detector:Traffic signs location updated to "+ str(len(self.lights_position))+ " elements")

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        if len(self.waypoints_np) == 0:
            rospy.logwarn("tl_detector: Waypoins numpy array is not initialized")
            return -1

        if type(pose) is list:
            get_distance = np.vectorize(lambda waypoint: np.linalg.norm(complex(pose[0], pose[1]) - waypoint))
        else:
            get_distance = np.vectorize(lambda waypoint: np.linalg.norm(complex(pose.position.x,
                                                                                pose.position.y) - waypoint))

        closest_point = get_distance(self.waypoints_np)
        return np.argmin(closest_point) # Get the index of the current value

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_wp = -1
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        # As the list of stop line points does not change, calculate it only once
        if self.pose:
            if len(self.nearest_waypoints_to_lines) == 0:
                for stop_line in stop_line_positions:
                    self.nearest_waypoints_to_lines = np.append(self.nearest_waypoints_to_lines,
                                                                self.get_closest_waypoint(stop_line))
                rospy.logwarn("tl_detector: Closest lines calculated")

            car_position = self.get_closest_waypoint(self.pose.pose)
            if self.car_last_wp is not None:
                if (self.car_last_wp - car_position) <= 0:
                    self.car_direction = 1
                else:
                    self.car_direction = -1

            self.car_last_wp = car_position

            # Now with the indexes of the nearest waypoints to a line, check if there is a point
            # which index is below the treshold. This works in both directions
            light_distances = [0 < (self.car_direction*(line_index - car_position)) < INDEX_DISTANCE_THRESHOLD
                               for line_index in self.nearest_waypoints_to_lines]
            self.last_in_range = self.in_range
            if any(light_distances):
                light_wp = self.nearest_waypoints_to_lines[np.argmax(light_distances)]
                self.in_range = True
                light = True
            else:
                self.in_range = False

            if self.in_range and not self.last_in_range:
                rospy.logwarn("tl_detector:Traffic light in range to detect. WP: " + str(light_wp) + " CarWP: "
                              + str(car_position))
                self.last_in_range = self.in_range
        else:
            rospy.logerr("tl_detector: Pose is not set")

        if light:
            # TODO: This is a stub, getting a fake state of the traffic light
            state = TrafficLight.RED   #self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
