#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float32
from PID_controller import PID_controller
from duckietown_msgs.msg import AprilTagDetectionArray, FSMState, Twist2DStamped

x = 0
error = 0
pid_c = 0
angular = 0



class lab4_angular_controller:
    def __init__(self):

        #Tuning
        self.KP = -4.5  # Increase kp until system oscillates once
        self.KI = 1.2   # Increase ki until system converges on desired value
        self.KD = 0.07  # Increase kd to reduce overshoot and settling time
        self.TIME_STEP = 0.1
        
        rospy.Subscriber("/KhimDuckieBot/fsm_node/mode", FSMState, self.callback)
        
        rospy.Subscriber("/KhimDuckieBot/apriltag_detector_node/detections", AprilTagDetectionArray, self.apriltag_callback)
        
        self.pub_angular = rospy.Publisher("/KhimDuckieBot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        
    def apriltag_callback(self, msg):
       
        global error 
        global pid_c
        global angular
        
        if len(msg.detections) == 0:
            rospy.loginfo("detections list is empty")
            angular = 0
        else:
            error = msg.detections[len(msg.detections)-1].transform.translation.x
            pid_c = PID_controller(KP=self.KP, KI=self.KI, KD=self.KD, TIME_STEP=self.TIME_STEP)
            angular = pid_c.PID(error)
            rospy.loginfo(rospy.get_caller_id() + " publish angular %f", angular)
       
    
    def callback(self, msg):

        global angular

        while msg.state == "LANE_FOLLOWING":
        
            self.velocity = 0
            self.omega = angular

            rospy.loginfo("angular: %f", angular) 
            rospy.loginfo("omega:  %f", self.omega)
       
            self.header = Header()
            self.header.seq = 0

            self.point_to_apriltag = Twist2DStamped(self.header, self.velocity, self.omega)
            self.pub_angular.publish(self.point_to_apriltag)

            rospy.sleep(0.5)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lab4_angular_controller", anonymous=True)

    lab4_angular_controller()

    rospy.spin()
