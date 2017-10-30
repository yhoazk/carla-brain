from styx_msgs.msg import TrafficLight
import rospy
import os
import cv2



class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        #rospy.logwarn("tl_classifier: Classification requested")

        ### Classification based on which part of the traffic light image (top, mid, or bottom) seems to be lit up

        self.img_h, self.img_w, self.img_d = image.shape

        height_trim = 0.1
        width_trim = 0.1

        self.l_channel = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)[int(height_trim*self.img_h):int((1.0 - height_trim)*self.img_h),int(width_trim*self.img_w):int((1.0-width_trim)*self.img_w),0]


        self.img_h, self.img_w = self.l_channel.shape

        self.top_third_marker = int(self.img_h / 3)
        self.bottom_third_marker = self.img_h - self.top_third_marker

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




        #evaluate which color is most likely

        max_count = max(count_result, key=count_result.get)

        if max_count == 'RED':
            rospy.loginfo("tl_classifier: RED light detected") 
            return TrafficLight.RED

        elif max_count == 'YELLOW':
            rospy.loginfo("tl_classifier: YELLOW light detected") 
            return TrafficLight.YELLOW

        elif max_count == 'GREEN':
            rospy.loginfo("tl_classifier: GREEN light detected")
            return TrafficLight.GREEN 

        else:
            rospy.logwarn("tl_classifier: ERROR - cannot classify light")
            return TrafficLight.UNKNOWN
