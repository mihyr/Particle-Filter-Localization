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


def measurement_model(x_t, y_t, theta_t, landmark, noise=True):
    """ This function defines the measurement model for two wheel planer mobile robot.
        Predicts range and heading of known landmarks based on current position of mobile robot
        assumption: global groundtruth location of static landmarks is known to robot

        Random gaussian noise created using std-dev data from ds0_Landmark_Groundtruth.dat for respective subject values

	Args: 
		x_t, y_t, theta_t: robot's current x,y, theta position and heading
        landmark (int: 6-20): number representing known landmarks i.e. subject 6 to 20 (not barcode)
        noise (bool): if true, it adds gausian noise to measurment model

	Returns: 
		expected_range,expected_bearing: returns expected/predicted range and heading
	"""
    #get data

    time_measured = measurement_data[0]
    barcode_no = measurement_data[1]
    range_measured = measurement_data[2]
    bearing_measured = measurement_data[3]
    
    subject_no = list(landmark_gt_data[0])

    index = subject_no.index(landmark)
    x_i = landmark_gt_data[1][index]
    y_i = landmark_gt_data[2][index]
    sd_x = landmark_gt_data[3][index]
    sd_y = landmark_gt_data[4][index]
    
    if noise == True:

        #add noise
        mu = 0
        
        sd_r = np.sqrt( sd_x**2 + sd_y**2 )
        sd_phi = np.atan2(sd_y,sd_x)
        
        r_cov = (np.random.normal(mu, sd_r))**2
        phi_cov = (np.random.normal(mu, sd_phi))**2

        x_cov = (np.random.normal(mu, sd_x))**2
        y_cov = (np.random.normal(mu, sd_y))**2
        
        # print(f'x covariance: {x_cov}, y ccovariance: {y_cov}')
        # print(r_cov,phi_cov)

    else:
        r_cov = 0
        phi_cov = 0
        x_cov = 0
        y_cov = 0

    # main calculation

    # expected_range = np.sqrt( (x_t - x_i)**2 + (y_t - y_i)**2 ) + r_cov
    # expected_bearing = np.arctan2((y_i -y_t) , (x_i - x_t)) - theta_t + phi_cov

    expected_range = np.sqrt( (x_t - x_i - x_cov)**2 + (y_t - y_i - y_cov)**2 ) 
    expected_bearing = np.arctan2((y_i -y_t + y_cov) , (x_i - x_t + x_cov)) - theta_t 

    return expected_range,expected_bearing