from styx_msgs.msg import TrafficLight
import rospy
import os
import cv2



class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        This implementation of the classifier uses OpenCV to determine color of a traffic light.
        Using CIELUV colorspace, the classifier extracts the lightness parameter of top, middle, and bottom areas of the image. 
        The area of the image with highest L value corresponds to the section of the traffic light that is currently lit up. 
        Using the LUV color space makes the classifier work reliably in various setups and light conditions, in both simulated and real-world environment. 
        For example, this classifier works reliably in light conditions that visually make the top light appear yellow/orange, where hue based classifiers are likely to fail.

        Note: 
            The classifier assumes the traffic light consists of 3 light bulbs, with RED being at the top, YELLOW in the middle, and GREEN at the botom.
            It requires adjustment to work in cases where the traffic light has different number of colors, or their arrangement is different (e.g., horizontal)

        Args:
            image (cv::Mat): image containing the traffic light, with just the 3 lights/bulbs, and cropped to include minimum background. 
            The classifier is agnostic to the resolution or proportions of the image 

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
  
        rospy.logdebug("tl_classifier: Classification requested")

        # An initial cropping can be applied to the image, to minimize the amount of background area outside of the traffic light. Tweak depending on the image source used.
        self.img_h, self.img_w, self.img_d = image.shape

        height_trim = 0.1
        width_trim = 0.1

        # Image is trimmed, converted to CIELUV and L channel is extracted
        self.l_channel = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)[int(height_trim*self.img_h):int((1.0 - height_trim)*self.img_h),int(width_trim*self.img_w):int((1.0-width_trim)*self.img_w),0]

        # Markers are established to enable splitting the image into top, mid, and bottom thirds
        self.img_h, self.img_w = self.l_channel.shape

        self.top_third_marker = int(self.img_h / 3)
        self.bottom_third_marker = self.img_h - self.top_third_marker

        # Magnitude of L is established for each section of the image
        top = 0
        mid = 0
        bottom = 0

        count_result = {'RED': 0 , 'YELLOW': 0, 'GREEN': 0}

        for i in range(self.top_third_marker):
            for j in range(self.img_w):
                top += self.l_channel[i][j]
        count_result['RED'] = top

        for i in range(self.top_third_marker, self.bottom_third_marker):
            for j in range(self.img_w):
                mid += self.l_channel[i][j]
        count_result['YELLOW'] = mid

        for i in range(self.bottom_third_marker, self.img_h):
            for j in range(self.img_w):
                bottom += self.l_channel[i][j]
        count_result['GREEN'] = bottom

        #The result is classified into one of the 3 colors and returned
        max_count = max(count_result, key=count_result.get)

        if max_count == 'RED':
            rospy.logdebug("tl_classifier: RED light detected") 
            return TrafficLight.RED

        elif max_count == 'YELLOW':
            rospy.logdebug("tl_classifier: YELLOW light detected") 
            return TrafficLight.YELLOW

        elif max_count == 'GREEN':
            rospy.logdebug("tl_classifier: GREEN light detected")
            return TrafficLight.GREEN 

        else:
            rospy.logwarn("tl_classifier: ERROR - cannot classify light")
            return TrafficLight.UNKNOWN
