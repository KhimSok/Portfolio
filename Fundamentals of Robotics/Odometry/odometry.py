#!/usr/bin/env python3

import rospy
from math import radians, sin, cos
from odometry_hw.msg import DistWheel, Pose2D



class hw6_node:
    def __init__(self):
        rospy.Subscriber("/dist_wheel", DistWheel, self.callback)
        self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)
        
        self.pre_x = 0
        self.pre_y = 0
        self.pre_theta = 0
        

    def callback(self, msg):
        
        self.L = 0.05 #baseline between wheels L = 0.05m
        self.SL = msg.dist_wheel_left
        self.SR = msg.dist_wheel_right
        self.total_S = (self.SL + self.SR)/2
        
        #calculate displacement
        self.total_theta = (self.SR - self.SL)/(2 * self.L)
        self.total_x_travel = self.total_S * cos(self.pre_theta + self.total_theta/2)
        self.total_y_travel = self.total_S * sin(self.pre_theta + self.total_theta/2)
        
        #calculate current x, y, theta pose
        self.current_x = self.pre_x + self.total_x_travel
        self.current_y = self.pre_y + self.total_y_travel
        self.current_theta = self.pre_theta + self.total_theta
        
        #store x, y, theta into a Pose2D type "current_pos"
        self.current_pos = Pose2D(self.current_x, self.current_y, self.current_theta)
        
        #publish current position
        self.pub.publish(self.current_pos)
        
        #update position, so it will become previous position when we callback next time
        self.pre_x = self.current_x
        self.pre_y = self.current_y
        self.pre_theta = self.current_theta
        
if __name__ == '__main__':
    rospy.init_node('hw6_node')
    hw6_node()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

