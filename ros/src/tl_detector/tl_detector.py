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




class TLDetector(object):
    """
    Traffic lights detector.

    When a traffic light waypoint is near car pose runs image detection and classification
    to determine state of the traffic light.

    Publishes to /traffic_waypoint index of waypoint with RED traffic light. If no RED traffic light
    in the vicinity publish -1.
    """
    def __init__(self):
        rospy.init_node('tl_detector')

        self.bridge = CvBridge()
        self.detector = TLDetectorSegmentation() # TLDetector that uses semantic segmentation
        self.classifier = TLClassifier()

        self.pose = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        self.base_waypoints_np = np.array([])
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)

        self.tl_config = yaml.safe_load(rospy.get_param("/traffic_light_config"))

        self.car_direction = 1
        self.last_car_wp_idx = None
        self.in_range = False
        self.last_in_range = False
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_tl_wp_idx = -1
        self.state_count = 0

        self.camera_image = None
        #rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        self.lights = []
        self.lights_position = []
        '''
        /vehicle/traffic_lights provides the location of the traffic light in 3D map
        space and gives ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)

        """ Publish the index of the waypoint nearest to the upcoming red traffic light"""
        self.traffic_waypoint_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.image_debug_pub = rospy.Publisher("/image_debug", Image, queue_size=1)

        self.loop()


    def loop(self):
        """main tl_detector message processing loop"""
        rate = rospy.Rate(4) # in Hz
        while not rospy.is_shutdown():
            rate.sleep()


    def pose_cb(self, msg):
        """Callback to receive pose"""
        self.pose = msg


    def get_light_state(self):
        """Detects traffic lights in self.camera_image.
        Returns single result of their classification.
        Publishes /image_debug with bounding boxes overlayed over detected TLs

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
            tl_class = self.classifier.get_classification(resized)
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
        self.image_debug_pub.publish(image_message)
        time_ms = int((timer() - start_time) * 1000)

        rospy.logwarn("tl_detector: detected {} TLs in img, {}/{} tf/tot ms, classes={}. result={}".format(
            len(bboxes), tf_ms, time_ms, classification, result))

        return result


    def calculate_closest_waypoint_idx(self, pose):
        """Identifies the index of closest waypoint to the given pose
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
            NB: 'closest' may be behind
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        if len(self.base_waypoints_np) == 0:
            rospy.logwarn("tl_detector: Waypoins numpy array is not initialized")
            return -1

        if type(pose) is list:
            # pose comes from stop_line_positions
            get_distance = np.vectorize(lambda waypoint: np.linalg.norm(complex(pose[0], pose[1]) - waypoint))
        else:
            # pose comes from ROS pose type
            get_distance = np.vectorize(lambda waypoint: np.linalg.norm(complex(pose.position.x,
                                                                                pose.position.y) - waypoint))

        closest_point = get_distance(self.base_waypoints_np)
        return np.argmin(closest_point)


    def get_next_tl_waypoint_index(self, stop_line_positions):
        """ Return index in base_waypoints of the next TL given car pose.

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
        """
        tl_wp_idx = -1
        if self.pose is None:
            rospy.logerr("tl_detector: Pose is not set")
            return tl_wp_idx
        if len(self.base_waypoints_np)==0:
            rospy.logerr("tl_detector: waypoints numpy array not set")
            return tl_wp_idx

        # find indices of waypoints for stop line positions (given by pairs like [1148.56, 1184.65])
        stop_lines_wp_idxs = [self.calculate_closest_waypoint_idx(stop_line_xy)
                                   for stop_line_xy in stop_line_positions]

        # find car waypoint index
        car_wp_idx = self.calculate_closest_waypoint_idx(self.pose.pose)
        if self.last_car_wp_idx is not None:
            if (self.last_car_wp_idx - car_wp_idx) <= 0:
                self.car_direction = 1
            else:
                self.car_direction = -1
        else:
            self.car_direction = 1
        self.last_car_wp_idx = car_wp_idx

        # Find the closest upcoming stop line waypoint index in front of the car
        INDEX_DISTANCE_THRESHOLD = 150
        min_dist = 1e+10
        wp1 = self.base_waypoints_np[car_wp_idx]
        for i in range(car_wp_idx, car_wp_idx+self.car_direction*INDEX_DISTANCE_THRESHOLD, self.car_direction):
            if np.isin(i, stop_lines_wp_idxs):
                wp2 = self.base_waypoints_np[i]
                dist = np.linalg.norm(wp1 - wp2)
                if dist < min_dist:
                    tl_wp_idx = i
                    min_dist = dist

        self.last_in_range = self.in_range
        if min_dist < 1e+10:
            self.in_range = True
        else:
            self.in_range = False

        if self.in_range and not self.last_in_range:
            rospy.logwarn("tl_detector: TL in range SL_WP: {}, Car_WP: {}".format(tl_wp_idx, car_wp_idx))
            self.last_in_range = self.in_range

        return tl_wp_idx


    def base_waypoints_cb(self, msg):
        """Callback to receive /base_waypoints"""
        waypoints_np = np.array([])

        for point in msg.waypoints:
            x_coord = point.pose.pose.position.x
            y_coord = point.pose.pose.position.y
            waypoints_np = np.append(waypoints_np, complex(x_coord, y_coord))

        self.base_waypoints_np = waypoints_np

        rospy.logwarn("tl_detector: updated {} base waypoints".format(len(self.base_waypoints_np)))


    def traffic_cb(self, msg):
        """Callback to receive ground truth traffic light states in simulator.
        Finds next closest traffic light in front, takes its ground truth state
        and publishes to /traffic_waypoint
        """
        self.lights = msg.lights
        self.lights_position = []
        for position in msg.lights:
            x = position.pose.pose.position.x
            y = position.pose.pose.position.y
            self.lights_position.append([x, y])

        tl_wp_idx = self.get_next_tl_waypoint_index(self.lights_position)
        state = TrafficLight.UNKNOWN

        if self.in_range:
            stop_lines_wp_idxs = [self.calculate_closest_waypoint_idx(stop_line_xy)
                                  for stop_line_xy in self.lights_position]
            for i in range(len(stop_lines_wp_idxs)):
                if tl_wp_idx == stop_lines_wp_idxs[i]:
                    state = self.lights[i].state
                    break
            if state==TrafficLight.RED:
                PUBLISH_STOP_LINE_OFFSET_IDX = 20
                tl_wp_idx -= PUBLISH_STOP_LINE_OFFSET_IDX
            else:
                tl_wp_idx = -1

        self.update_state_and_publish(state, tl_wp_idx)
        rospy.logwarn("tl_detector: traffic_cb, tl_wp_idx={}, state={}".format(tl_wp_idx, state))


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        tl_wp_idx = self.get_next_tl_waypoint_index(self.tl_config['stop_line_positions'])
        rospy.logwarn("tl_detector: image_cb next_wp {}".format(tl_wp_idx))

        if tl_wp_idx > -1:
            #state = self.get_light_state()
            state = TrafficLight.RED
            self.update_state_and_publish(state, tl_wp_idx)


    def update_state_and_publish(self, state, tl_wp_idx):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 1
            self.state = state
        else:
            self.state_count += 1

        STATE_COUNT_THRESHOLD = 1
        if self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if self.state == TrafficLight.RED:
                self.last_tl_wp_idx = tl_wp_idx
            else:
                self.last_tl_wp_idx = -1
            self.traffic_waypoint_pub.publish(Int32(self.last_tl_wp_idx))





if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
