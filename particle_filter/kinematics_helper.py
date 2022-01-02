#!/usr/bin/env python3
# Robot kinematics helper functions
import numpy as np
import math

def motion_model(v,w,x_1,y_1,theta_1, dt):
    """ This function defines the motion model for two wheel planer mobile robot.
        Predicts current position and heading.
	Args: 
		v: linear velocity of robot in m/s
        w: angular velocity of robot in rad/s
        x_1, y_1, theta_1: robot x,y,theta position and heading at time t-1 (i.e. previous position and heading)
        dt: time difference

	Returns: 
		x_t, y_t, theta_t: robot's current x,y, theta position and heading
	"""
    #eqn from PR book
    
    
    if w ==0:
        #for zero angular velocity    
        x_t = v * np.cos(theta_1) * dt + x_1
        
        y_t = v * np.sin(theta_1) * dt + y_1
        
        theta_t = theta_1

    else:
        #for non-zero angular velocity
        theta_t = theta_1 + w * dt

        x_t = (v / w) * (-np.sin(theta_1) + np.sin(theta_t)) + x_1

        y_t = (v / w) * (np.cos(theta_1) - np.cos(theta_t)) + y_1

    return x_t , y_t , theta_t


def landmark_estimate(particle,landmark, noise=True):
    ld_x = particle[0] + landmark[0]
    ld_y = particle[1] + landmark[1]
    return ld_x,ld_y
