#!/usr/bin/env python3

class PID_controller:
    def __init__(self, KP, KI, KD, TIME_STEP):
        
        self.kp = KP
        self.ki = KI
        self.kd = KD
        
        self.dt = TIME_STEP
        self.e_integrate = 0
        self.e_prev = 0
        
        self.pid = [0, 0, 0]
        self.output = 0
        

    def PID(self, e):
    
        self.e_integrate += e

        if self.e_prev == None:
            e_dot = 0.0
        else:
            e_dot = (e - self.e_prev) / self.dt
        
        P = self.kp * e
        I = self.ki * self.e_integrate
        D = self.kd * e_dot
        
        self.e_prev = e
        self.pid = [P, I, D]
        self.output = P + I + D
        
        return self.output
