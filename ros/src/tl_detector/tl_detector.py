#!/usr/bin/env python
"""
Traffic Light Detector ROS node for Carla.

Uses tensorflow to run semantic segmentation model (FCN).
The model finds bounding boxes for potential traffic lights in the image.
Then classifier is called to classify what kind of light is it.
The waypoint index of the next traffic light (within range) is published to /traffic_waypoint.
If next traffic light is out of range or in-range but not RED then -1 is published.

Implementation uses separate thread to run detector/tensorflow code because it is resource
intensive and interferes with message dispatch for ROS if run in the same thread.
"""
from threading import Lock, Thread, Event
from timeit import default_timer as timer
import yaml

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, TrafficLight, TrafficLightArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

from light_classification.tl_classifier import TLClassifier
from tl_detector_segmentation import TLDetectorSegmentation


class TLDetector(object):
    """
    Traffic lights detector.

    When a traffic light waypoint is near car pose runs image detection and classification
    to determine state of the traffic light.
    Alternatively can use /vehicle/traffic_lights for 'fake' implementation when simulator
    provides data on state of traffic lights.

    Publishes to /traffic_waypoint index of next waypoint with RED traffic light.
    If no RED traffic light in the vicinity publish -1.
    """
    def __init__(self):
        rospy.init_node('tl_detector')

        # Lock to synchronise access to image data between 2 threads
        self.lock = Lock()
        # Missed images count. if detector thread is busy we keep consuming images from the topic
        # but just discard them
        self.missed_images = -1
        # Event to be raised from ROS thread to detector thread that there is image to be processed.
        # Detector thread just waits for the event.
        self.event = Event()
        self.event.clear()
        # Create and start separate detector thread that detects/classifies data in images
        # and publishes to /traffic_waypoint
        self.thread = Thread(target=self.detector_thread)
        self.thread.start()
        # This main thread handles callbacks from ROS

        # Machinery to deal with images/detection/segmentation
        self.bridge = CvBridge()
        self.detector = TLDetectorSegmentation()  # TLDetector that uses semantic segmentation
        self.classifier = TLClassifier()

        # Subscribe to receive car pose
        self.pose = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        # Subscribe to receive base waypoints (essentially the planned route)
        # With Carla/this project it is published just once. So we cache it.
        self.base_waypoints_np = np.array([])
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)

        # Read/cache positions of traffic lights along the route.
        self.tl_config = yaml.safe_load(rospy.get_param("/traffic_light_config"))

        # Direction of car. +1 == waypoint indexes increase as car moves.
        self.car_direction = 1
        # Last nearest waypoint index of the car.
        self.last_car_wp_idx = None
        # Do we have traffic lights potentially in range for detection in the image?
        self.in_range = False
        self.last_in_range = False
        # Detected traffic light state.
        self.state = TrafficLight.UNKNOWN
        self.state_count = 0
        self.last_state = TrafficLight.UNKNOWN
        # Last detected index of RED oncoming traffic light.
        self.last_tl_wp_idx = -1
        # Cached positions of stop lines in front of traffic lights.
        self.stop_lines_wp_idxs = []

        # Camera image subscription.
        self.has_image = False
        self.camera_image = None
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        # /vehicle/traffic_lights provides the location of the traffic light in 3D map
        # space in simulator. It gives ground truth data source for the traffic light
        # classifier by sending the current color state of all traffic lights in the
        # simulator. When testing on the vehicle, the color state will not be available.
        self.lights = []
        self.lights_position = []
        # For development in simulator uncomment the next line and comment out the image_cb line above.
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)

        # Publish the index of the waypoint nearest to the upcoming red traffic light.
        self.traffic_waypoint_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        # Publish debug image with detected traffic lights and bounding boxes around them.
        self.image_debug_pub = rospy.Publisher("/image_debug", Image, queue_size=1)

        # This thread keeps taking messages from ROS until shutdown.
        rospy.spin()
        # Wait for 5 seconds for detector thread once this thread is stopped.
        self.thread.join(timeout=5)


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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # detect bounding boxes of what looks like traffic lights
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

            # Skip small images to avoid false positives
            MIN_IMAGE_HEIGHT = 50
            if tl_image.shape[0] < MIN_IMAGE_HEIGHT:
                rospy.loginfo("tl_detector: TL image detected too small and likely a false positive. Discarding & continuing.")
                continue
   
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

        # come to consensus about the state of traffic lights in the picture
        result = TrafficLight.UNKNOWN
        if classification[TrafficLight.RED] > 0:
            result = TrafficLight.RED
        elif classification[TrafficLight.YELLOW] > 0:
            result = TrafficLight.YELLOW
        elif classification[TrafficLight.GREEN] > 0:
            result = TrafficLight.GREEN

        # publish debug image message
        resized = cv2.resize(tl_img_debug, (400,300,), interpolation=cv2.INTER_LINEAR)
        image_message = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")
        self.image_debug_pub.publish(image_message)
        time_ms = int((timer() - start_time) * 1000)

        rospy.logwarn("tl_detector: detected {} TLs in img, {}/{} tf/tot ms, result={}".format(
            len(bboxes), tf_ms, time_ms, result))

        return result


    def calculate_closest_waypoint_idx(self, pose):
        """Identify the index of closest waypoint in self.base_waypoints_np for the given pose
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
            NB: 'closest' may be behind
        Args:
            pose (Pose): position to match a waypoint to. Either ROS pose type or complex(x,y)

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        if len(self.base_waypoints_np) == 0:
            rospy.logwarn("tl_detector: Waypoints numpy array is not initialized")
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
        """ Return index in base_waypoints of the next traffic light in front, given the car pose
        in self.pose

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
        """
        tl_wp_idx = -1
        if self.pose is None:
            rospy.logdebug("tl_detector: Pose is not set")
            return tl_wp_idx
        if len(self.base_waypoints_np)==0:
            rospy.logdebug("tl_detector: waypoints numpy array not set")
            return tl_wp_idx

        if len(self.stop_lines_wp_idxs)==0:
            # find indices of waypoints for stop line positions (given by pairs like [1148.56, 1184.65])
            # Do it only once.
            # Assume the they never change, at least in this project.
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
        else:
            self.in_range = False

        if self.in_range and not self.last_in_range:
            rospy.logwarn("tl_detector: TL in range StopLine_WP: {}, Car_WP: {}".format(tl_wp_idx, car_wp_idx))
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
        """Loop that runs in separate thread. Identifies RED lights in the camera image
        and publishes the index of the waypoint of the closest light's stop line to
        /traffic_waypoint topic.
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

            rospy.logwarn("tl_detector: detector_thread next_wp {}, {}ms: missed imgs {}".format(
                tl_wp_idx, wp_time, missed_images))

            if tl_wp_idx > -1:
                # In range of traffic light, run image detection
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
            msg (Image): image from car-mounted camera or from simulator
        """
        self.lock.acquire()
        self.has_image = True
        self.camera_image = msg
        self.missed_images += 1
        self.event.set()
        self.lock.release()


    def update_state_and_publish(self, state, tl_wp_idx):
        """
        Publish waypoint index (in /base_waypoints) of upcoming RED light to /traffic_waypoints.

        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is used.
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
