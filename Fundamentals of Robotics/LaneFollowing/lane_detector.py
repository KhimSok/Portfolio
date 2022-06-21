#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class lab3_lane_detector:
    def __init__(self):
    
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        
        rospy.Subscriber("/KhimDuckieBot/camera_node/image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buff_size=2**24)
        
        self.pub_image_line_detection = rospy.Publisher("debug/segments/image_lines_white_and_yellow", Image, queue_size=10)
        
        
    def lanefilter_cb(self, msg):
        
        compressed_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(compressed_img, image_size, interpolation=cv2.INTER_NEAREST)
        cropped_image = new_image[offset:, :]
        
        cropped_image_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        
        img_hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        img_white = cv2.inRange(img_hsv, (0,0,150),(255,35,255))
        img_yellow = cv2.inRange(img_hsv, (0,90,200),(45,255,255))

        kernal_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        image_erode_white = cv2.erode(img_white, kernal_erode) 
        image_erode_yellow = cv2.erode(img_yellow, kernal_erode)
        mask_erode_white = cv2.bitwise_or(image_erode_white, image_erode_white)
        mask_erode_yellow = cv2.bitwise_or(image_erode_yellow, image_erode_yellow)
        
        kernal_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
        image_dilate_white = cv2.dilate(image_erode_white, kernal_dilate)
        image_dilate_yellow = cv2.dilate(image_erode_yellow, kernal_dilate)
        mask_dilate_white = cv2.bitwise_or(image_dilate_white, image_dilate_white)
        mask_dilate_yellow = cv2.bitwise_or(image_dilate_yellow, image_dilate_yellow)
        
        mask_white = cv2.bitwise_or(image_dilate_white, mask_dilate_white)
        mask_yellow = cv2.bitwise_or(image_dilate_yellow, mask_dilate_yellow)
        
        output_white = cv2.bitwise_and(cropped_image, cropped_image, mask=mask_white)
        output_yellow = cv2.bitwise_and(cropped_image, cropped_image, mask=mask_yellow)
        
        
        
        cv_canny_img = cv2.Canny(cropped_image_gray, 100, 200)
        ros_canny_img = self.bridge.cv2_to_imgmsg(cv_canny_img, "mono8")
        
        
        output_white = cv2.cvtColor(output_white, cv2.COLOR_BGR2GRAY)
        output_yellow = cv2.cvtColor(output_yellow, cv2.COLOR_BGR2GRAY)
        
        cv_white_edge = cv2.bitwise_and(output_white, cv_canny_img)
        cv_yellow_edge = cv2.bitwise_and(output_yellow, cv_canny_img)
        
        rho = 1
        theta = numpy.pi/180
        threshole = 5
        minLineLength = 3
        maxLineGap = 1
        
        cv_hough_trans_white = cv2.HoughLinesP(cv_white_edge,rho,theta,threshole,minLineLength,maxLineGap)
        cv_hough_trans_yellow = cv2.HoughLinesP(cv_yellow_edge,rho,theta,threshole,minLineLength,maxLineGap)
        
        cv_line_white_output = self.output_lines(cropped_image, cv_hough_trans_white)
        cv_line_yellow_output = self.output_lines(cropped_image, cv_hough_trans_yellow)
        cv_white_and_yellow_line = cv2.bitwise_or(cv_line_white_output, cv_line_yellow_output)
        
        ros_line_white_and_yellow_output = self.bridge.cv2_to_imgmsg(cv_white_and_yellow_line, "bgr8")
        
        rospy.loginfo(rospy.get_caller_id() + " %s", compressed_img)
        
        self.pub_image_line_detection.publish(ros_line_white_and_yellow_output)
        
        
    def output_lines(self, original_image, lines):
        output = numpy.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lab3_lane_detector", anonymous=True)

    ros_img = lab3_lane_detector()

    rospy.spin()
