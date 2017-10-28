from styx_msgs.msg import TrafficLight
import rospy
import numpy as np
#import matplotlib.pyplot as plt
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

        rospy.logwarn("tl_classifier: Classification requested")

        l_channel = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)[:,:,0]

        img_h, img_w = l.shape

        top_third_marker = int(img_h / 3)
        bottom_third_marker = img_h - top_third_marker

        top = 0
        mid = 0
        bottom = 0

        count_result = {'RED': 0 , 'YELLOW': 0, 'GREEN': 0}

        for i in range(top_third_marker):
            for j in range(img_w):
                top += l[i][j]
        count_result['RED'] = top
        

        for i in range(top_third_marker, bottom_third_marker):
            for j in range(img_w):
                mid += l[i][j]
        count_result['YELLOW'] = mid


        for i in range(bottom_third_marker, img_h):
            for j in range(img_w):
                bottom += l[i][j]
        count_result['GREEN'] = bottom


        #evaluate which color is most likely

        max_count = max(d, key=d.get)

        if max_count == 'RED':
            return TrafficLight.RED
            rospy.logwarn("tl_classifier: RED light detected") 

        elif max_count == 'YELLOW':
            return TrafficLight.YELLOW
            rospy.logwarn("tl_classifier: YELLOW light detected") 

        elif max_count == 'GREEN':
            return TrafficLight.GREEN
            rospy.logwarn("tl_classifier: RED light detected") 

        else:
            return TrafficLight.UNKNOWN
            rospy.logwarn("tl_classifier: ERROR - cannot classify light")

