import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement):
    # In this function you implement your estimator. The function arguments
    # are:
    #  time: current time in [s] 
    #  dt: current time step [s]
    #  internalStateIn: the estimator internal state, definition up to you. 
    #  steeringAngle: the steering angle of the bike, gamma, [rad] 
    #  pedalSpeed: the rotational speed of the pedal, omega, [rad/s] 
    #  measurement: the position measurement valid at the current time step
    #
    # Note: the measurement is a 2D vector, of x-y position measurement.
    #  The measurement sensor may fail to return data, in which case the
    #  measurement is given as NaN (not a number).
    #
    # The function has four outputs:
    #  x: your current best estimate for the bicycle's x-position
    #  y: your current best estimate for the bicycle's y-position
    #  theta: your current best estimate for the bicycle's rotation theta
    #  internalState: the estimator's internal state, in a format that can be understood by the next call to this function
    # this internal state needs to correspond to your init function:

    #Internal state consists of [x, y, theta, r, B, P_m, w, gamma]

    x = internalStateIn[0]
    y = internalStateIn[1]
    theta = internalStateIn[2]
    r = internalStateIn[3]
    B = internalStateIn[4]
    P_m = internalStateIn[5]
    w = pedalSpeed
    gamma = steeringAngle


    #Check for new measurement
    newMeasurement = False 
    if not (np.isnan(measurement[0]) or np.isnan(measurement[1])):
        newMeasurement = True
        z = np.matrix([measurement[0], measurement[1]]).T


    #### Prediction step ####
    x_m = np.matrix([x, y, theta, r, B,]).T

    Sigma_vv = np.matrix([[0.05, 0, 0, 0, 0], 
                        [0, 0.05, 0, 0, 0],
                        [0, 0, 0.07, 0, 0],
                        [0, 0, 0, 0.01, 0], 
                        [0, 0, 0, 0, 0.01]])

    #Linearize to find A matrix
    A = np.eye(5) + dt*np.matrix([[0, 0, -5*r*w*np.sin(theta), 0, 0], 
                                [0, 0, 5*r*w*np.cos(theta), 0, 0],
                                [0, 0, 0, 0, 0],
                                [0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 1]])

    L = dt*np.eye(5)
    

    #Predict x_p and P_p
    x_p = x_m + dt*np.matrix([[5*w*r*np.cos(theta)], 
                            [5*w*r*np.sin(theta)], 
                            [(5*w*r/B)*np.tan(gamma)], 
                            [0], 
                            [0]])
    P_p = A @ P_m @ A.T + L @ Sigma_vv @ L.T


    #### Measurement update ####

    #If there is a new measuremnt, perform the measurement update step 
    if(newMeasurement):

        #Extract states from x_p
        x = x_p[0, 0]
        y = x_p[1, 0]
        theta = x_p[2, 0]
        r = x_p[3, 0]
        B = x_p[4, 0]


        Sigma_ww = np.matrix([[1, 0], 
                            [0, 1]])
        
        #Measurement model
        h = np.matrix([[x + (1/2)*B*np.cos(theta)], 
                        [y + (1/2)*B*np.sin(theta)]])
        
        #Linearlize to find H
        H = np.matrix([[1, 0, -(1/2)*B*np.sin(theta), 0, 0],
                        [0, 1, (1/2)*B*np.cos(theta), 0, 0]])
        M = np.eye(2)


        #Perform meaurement update
        K = P_p @ H.T @ np.linalg.inv(H @ P_p @ H.T + M @ Sigma_ww @ M.T)
        x_m = x_p + K @ (z - h)
        P_m = (np.eye(5) - K @ H) @ P_p

    #If no new measurement, set the posterior equal to the predicted state and variance. 
    else: 
        x_m = x_p
        P_m = P_p

    #### OUTPUTS ####
    # Update the internal state (will be passed as an argument to the function
    # at next run), must obviously be compatible with the format of
    # internalStateIn:
    
    x = x_m[0, 0]
    y = x_m[1, 0]
    theta = x_m[2, 0]
    r = x_m[3, 0]
    B = x_m[4, 0]

    internalStateOut = [x,
                     y,
                     theta,
                     r, 
                     B, 
                     P_m,
                     ]

    # DO NOT MODIFY THE OUTPUT FORMAT:
    return x, y, theta, internalStateOut 


