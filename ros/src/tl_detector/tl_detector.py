#!/usr/bin/env python
"""
Traffic Light Detector node for Carla
"""
from timeit import default_timer as timer

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf

import cv2
import yaml
import numpy as np

from light_classification.tl_classifier import TLClassifier
from tl_detector_segmentation import TLDetectorSegmentation


STATE_COUNT_THRESHOLD = 1
INDEX_DISTANCE_THRESHOLD = 150


class TLDetector(object):
    """
    Traffic lights detector.
    When a traffic light waypoint is near car pose runs image detection and classification
    to determine state of the traffic light.
    Publishes to /traffic_waypoint
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

        # create TLDetector that uses semantic segmentation
        # underneath it creates tensorflow session and loads pre-trained graph
        self.detector = TLDetectorSegmentation()

        """ Publish the index of the waypoint nearest to the upcoming red traffic light
        """
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map
        space and helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available.
        You'll need to rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        self.pub_debug = rospy.Publisher("/image_debug", Image, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        rospy.spin()


    def pose_cb(self, msg):
        """Callback to receive pose"""
        self.pose = msg


    def waypoints_cb(self, waypoints):
        """Callback to receive waypoints"""
        waypoints_np = np.array([])

        for point in waypoints.waypoints:
            x_coord = point.pose.pose.position.x
            y_coord = point.pose.pose.position.y
            waypoints_np = np.append(waypoints_np, complex(x_coord, y_coord))

        self.waypoints = waypoints
        self.waypoints_np = waypoints_np

        rospy.logwarn("Waypoints updated to "+ str(len(self.waypoints_np))+ " elements")


    def traffic_cb(self, msg):
        """Callback to receive ground truth traffic light states in simulator"""
        self.lights = msg.lights
        if len(msg.lights) != len(self.lights_position):
            for position in msg.lights:
                x = position.pose.pose.position.x
                y = position.pose.pose.position.y
                self.lights_position = np.append(self.lights_position, complex(x, y))

            rospy.logwarn("tl_detector: Traffic lights locations updated: count {}".format(len(self.lights_position)))


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


    def get_light_state(self):
        """Detects traffic lights in self.camera_image and returns single result of their classification

        Args:
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.has_image:
            return TrafficLight.UNKNOWN

        start_time = timer()

        # detect bounding boxes of what looks like traffic lights
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        bboxes, tf_ms = self.detector.detect(cv_image)

        # extract TL images and classify. create debug segmented image
        classification = {TrafficLight.UNKNOWN: 0,
                          TrafficLight.RED: 0,
                          TrafficLight.YELLOW: 0,
                          TrafficLight.GREEN: 0}
        tl_img_debug = np.copy(cv_image)
        rect_img = np.zeros_like(tl_img_debug)
        for i, box in enumerate(bboxes):
            x1 = box[0][0]
            y1 = box[0][1]
            x2 = box[1][0]
            y2 = box[1][1]
            tl_image = cv_image[y1:y2, x1:x2]
            classifier_size = (128,128)
            resized = cv2.resize(tl_image, classifier_size, cv2.INTER_LINEAR)
            # Classification
            tl_class = self.light_classifier.get_classification(resized)
            classification[tl_class] += 1
            # debug output
            color = [255,255,255]
            if tl_class == TrafficLight.RED:
                color = [0, 0, 255]
            elif tl_class == TrafficLight.YELLOW:
                color = [0, 255, 255]
            elif tl_class == TrafficLight.GREEN:
                color = [0, 255, 0]
            cv2.rectangle(rect_img, box[0], box[1], color, thickness=-1)
        cv2.addWeighted(tl_img_debug, 1.0, rect_img, 0.5, 0, tl_img_debug)

        # come to consensus what we are looking at
        result = TrafficLight.UNKNOWN
        if classification[TrafficLight.RED] > 0:
            result = TrafficLight.RED
        elif classification[TrafficLight.YELLOW] > 0:
            result = TrafficLight.YELLOW
        elif classification[TrafficLight.GREEN] > 0:
            result = TrafficLight.GREEN

        # publish debug message
        resized = cv2.resize(tl_img_debug, (400,300,), interpolation=cv2.INTER_LINEAR)
        image_message = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")
        self.pub_debug.publish(image_message)
        time_ms = int((timer() - start_time) * 1000)

        rospy.logwarn("tl_detector: detected {} TLs in img, {}/{} tf/tot ms, classes={}. result={}".format(
            len(bboxes), tf_ms, time_ms, classification, result))

        return result


    def process_traffic_lights(self):
        """ High level logic: close to traffic light waypoint? if so, determine from image what its state

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
                rospy.logwarn("tl_detector: Closest stop positions calculated")

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
                rospy.logwarn("tl_detector: Traffic light in range to detect. WP: " + str(light_wp) + " CarWP: "
                              + str(car_position))
                self.last_in_range = self.in_range
        else:
            rospy.logerr("tl_detector: Pose is not set")

        if light:
            state = self.get_light_state()
            return light_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
