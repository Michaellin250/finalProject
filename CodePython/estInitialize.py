import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estInitialize():
    # Fill in whatever initialization you'd like here. This function generates
    # the internal state of the estimator at time 0. You may do whatever you
    # like here, but you must return something that is in the format as may be
    # used by your estRun() function as the first returned variable.
    #
    # The second returned variable must be a list of student names.
    # 
    # The third return variable must be a string with the estimator type
    
    # Inital chosen state values based in information in the project doc
    x = 0
    y = 0
    theta = np.pi/4 #North East
    r = 0.425 #nominal vaule
    B = 0.8 #nominal value
    P_m = np.matrix([[1, 0, 0, 0, 0], 
                    [0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])

    internalState = [x,
                     y,
                     theta,
                     r, 
                     B,
                     P_m,
                     ]

    # replace these names with yours. Delete the second name if you are working alone.
    studentNames = ['Michael Lin',
                    'Svein Jostein Husa']
    
    # replace this with the estimator type. Use one of the following options:
    #  'EKF' for Extended Kalman Filter
    #  'UKF' for Unscented Kalman Filter
    #  'PF' for Particle Filter
    #  'OTHER: XXX' if you're using something else, in which case please
    #                 replace "XXX" with a (very short) description
    estimatorType = 'EKF'  
    
    return internalState, studentNames, estimatorType

