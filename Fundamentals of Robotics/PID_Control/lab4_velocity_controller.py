#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float32
from PID_controller import PID_controller
from duckietown_msgs.msg import AprilTagDetectionArray, FSMState, Twist2DStamped


error = 0
pid_c = 0
velocity = 0
angular = 0



class lab4_velocity_controller:
    def __init__(self):
        
        #Tuning
        self.KP = -10	# Increase kp until system oscillates once
        self.KI = 0.05   # Increase ki until system converges on desired value
        self.KD = 0.58  # Increase kd to reduce overshoot and settling time
        self.TIME_STEP = 0.1
        
        rospy.Subscriber("/KhimDuckieBot/fsm_node/mode", FSMState, self.callback)
        
        rospy.Subscriber("/KhimDuckieBot/apriltag_detector_node/detections", AprilTagDetectionArray, self.apriltag_callback)
        
        self.pub_velocity = rospy.Publisher("/KhimDuckieBot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        rospy.Subscriber("/KhimDuckieBot/lane_controller_node/car_cmd", Twist2DStamped, self.omega_callback)
        
    def omega_callback(self, msg):
        global angular
        angular = msg.omega
        
        
    def apriltag_callback(self, msg):
       
        global error
        global pid_c
        global velocity
        
        if len(msg.detections) == 0:
            rospy.loginfo("detections list is empty")
            velocity = 0
            angular = 0
        else:
            error = msg.detections[len(msg.detections)-1].transform.translation.y
            pid_c = PID_controller(KP=self.KP, KI=self.KI, KD=self.KD, TIME_STEP=self.TIME_STEP)
            velocity = pid_c.PID(error)
            rospy.loginfo(rospy.get_caller_id() + " publish velocity %f", velocity)
       
    
    def callback(self, msg):

        global velocity
        global angular

        while msg.state == "LANE_FOLLOWING":
        
            self.vel = velocity
            self.omega = angular

            rospy.loginfo("velocity: %f", velocity) 
       
            self.header = Header()
            self.header.seq = 0

            self.drive_to_apriltag = Twist2DStamped(self.header, self.vel, self.omega)
            self.pub_velocity.publish(self.drive_to_apriltag)

            rospy.sleep(1)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lab4_velocity_controller", anonymous=True)

    lab4_velocity_controller()

    rospy.spin()
