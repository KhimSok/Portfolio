#!/usr/bin/env python3

import rospy
from math import radians, sin, cos, pi
from std_msgs.msg import Header
from duckietown_msgs.msg import FSMState, Twist2DStamped


class lab2_circle:
    def __init__(self):
        
        rospy.Subscriber("/KhimDuckieBot/fsm_node/mode", FSMState, self.callback)
        
        self.pub = rospy.Publisher("/KhimDuckieBot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        

    def callback(self, msg):
        

        if msg.state == "LANE_FOLLOWING":
                            
            self.velocity = 0.5
            self.radius = 0.5
        
            self.circumference = 2 * pi * self.radius
        
            #self.time = (self.circumference/self.velocity)*3
            self.time = 15
            self.omega = (self.velocity/self.radius)*2.3
            self.curr_secs = int(rospy.get_time())
            self.goal_secs = int(rospy.get_time() + self.time)
        
            self.header = Header()
            self.header.seq = 0
        
            self.curr_secs = int(rospy.get_time())
            self.goal_secs = int(rospy.get_time() + self.time)
            
            self.header.stamp.secs = int(rospy.get_time() + self.time)
            self.circle = Twist2DStamped(self.header, self.velocity, self.omega)
            self.pub.publish(self.circle)
            
            while self.curr_secs < self.header.stamp.secs:
                self.curr_secs = int(rospy.get_time())
                rospy.loginfo(rospy.get_caller_id() + " time is %d, header.secs is %d, header.seq is %d", self.curr_secs, self.header.stamp.secs, self.header.seq)
                if self.curr_secs == self.goal_secs:
                    self.velocity = 0
                    self.omega = 0    
                    self.circle = Twist2DStamped(self.header, self.velocity, self.omega)
            self.pub.publish(self.circle)
            rospy.loginfo(rospy.get_caller_id() + " Drive circle 1m completed, header.seq is %d", self.header.seq)
        
if __name__ == '__main__':
    rospy.init_node('lab2_circle')
    lab2_circle()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

