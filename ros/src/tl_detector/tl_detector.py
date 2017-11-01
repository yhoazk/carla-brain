#!/usr/bin/env python
"""
Traffic Light Detector node for Carla
"""
from threading import Lock, Thread, Event

from timeit import default_timer as timer
import yaml
import numpy as np

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, TrafficLight
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

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

        self.lock = Lock()
        self.missed_images = -1
        self.event = Event()
        self.event.clear()
        self.thread = Thread(target=self.detector_thread)
        self.thread.start()

        self.has_image = None

        self.bridge = CvBridge()
        self.detector = TLDetectorSegmentation()  # TLDetector that uses semantic segmentation
        self.classifier = TLClassifier()

        self.pose = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.has_image = False
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
        self.stop_lines_wp_idxs = []
        self.state_count = 0

        self.camera_image = None
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        self.lights = []
        self.lights_position = []
        '''
        /vehicle/traffic_lights provides the location of the traffic light in 3D map
        space and gives ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available.
        '''
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)

        """ Publish the index of the waypoint nearest to the upcoming red traffic light"""
        self.traffic_waypoint_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.image_debug_pub = rospy.Publisher("/image_debug", Image, queue_size=1)

        rospy.spin()
        self.thread.join(timeout=5)

    def pose_cb(self, msg):
        """Callback to receive pose
        Pose may be published very often, so we just store it here without any calculation
        """
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
   
            # Classification
            rospy.logdebug("tl_detector:About to call classifier")
            tl_class = self.classifier.get_classification(tl_image)
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

        if len(self.stop_lines_wp_idxs)==0:
            # find indices of waypoints for stop line positions (given by pairs like [1148.56, 1184.65])
            # do it only once. assume the they never change
            self.stop_lines_wp_idxs = [self.calculate_closest_waypoint_idx(stop_line_xy)
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

        light_dist = [0 < (self.car_direction *(line_index - car_wp_idx)) < INDEX_DISTANCE_THRESHOLD
                        for line_index in self.stop_lines_wp_idxs]
        
        self.last_in_range = self.in_range

        if any(light_dist):
            tl_wp_idx = self.stop_lines_wp_idxs[np.argmax(light_dist)]
            self.in_range = True
            light = True
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
        start = timer()

        self.lights = msg.lights
        self.lights_position = []
        for position in msg.lights:
            x = position.pose.pose.position.x
            y = position.pose.pose.position.y
            self.lights_position.append([x, y])

        tl_wp_idx = self.get_next_tl_waypoint_index(self.lights_position)

        state = TrafficLight.UNKNOWN

        if self.in_range:
            for i in range(len(self.stop_lines_wp_idxs)):
                if tl_wp_idx == self.stop_lines_wp_idxs[i]:
                    state = self.lights[i].state
                    break
            if state == TrafficLight.RED:
                PUBLISH_STOP_LINE_OFFSET_IDX = 20
                tl_wp_idx -= PUBLISH_STOP_LINE_OFFSET_IDX
            else:
                tl_wp_idx = -1

        self.update_state_and_publish(state, tl_wp_idx)
        rospy.logwarn("tl_detector: traffic_cb, tl_wp_idx={}, state={}, {}ms".format(tl_wp_idx, state,
                                                                                     int(float(timer()-start)*1000.)))

    def detector_thread(self):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        """
        while not rospy.is_shutdown() and self.event.wait():
            self.lock.acquire()
            self.event.clear()
            missed_images = self.missed_images
            self.missed_images = -1
            self.lock.release()
            start = timer()

            tl_wp_idx = self.get_next_tl_waypoint_index(self.tl_config['stop_line_positions'])
            wp_time = int(float(timer()-start)*1000.)

            rospy.logwarn("tl_detector: detector_thread next_wp {}, {}ms: missed images {}".format(tl_wp_idx, wp_time, missed_images))

            if tl_wp_idx > -1:
                start = timer()
                state = self.get_light_state()
                img_time = int(float(timer() - start) * 1000.)
                self.update_state_and_publish(state, tl_wp_idx)
                rospy.logwarn("tl_detector: detector_thread state={}, {}ms".format(state, img_time))
            else:
                self.update_state_and_publish(TrafficLight.RED, -1)


    def image_cb(self, msg):
        """Incoming camera image callback

        Args:
            msg (Image): image from car-mounted camera

        """
        self.lock.acquire()
        self.has_image = True
        self.camera_image = msg
        self.missed_images += 1
        self.event.set()
        self.lock.release()

    def update_state_and_publish(self, state, tl_wp_idx):
        """
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        """
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
