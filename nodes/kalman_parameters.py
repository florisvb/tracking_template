import numpy as np

### Define kalman filter properties ########

'''

nstates ------- number of states
nmeasurements - number of measurements
x0  ----------- initial state
P0  ----------- initial covariance
phi    -------- drift dynamics model (e.g. A in xdot = Ax + Bu)  
gamma  -------- control dynamics model (e.g. B in xdot = Ax + Bu)
H  ------------ observation model, maps states to observations (e.g. C in y = Cx + Du)
Q  ------------ state covariance
R  ------------ measurement covariance
gammaW  ------- observation model, maps states to observations (e.g. D in y = Cx + Du)
        

x = | x    |
    | xdot |
    | y    |
    | ydot |

xdot = phi*x + gamma*u
y = H*x + gammaW*u

'''
        
phi = np.matrix([                    [1, 1, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 1],
                                     [0, 0, 0, 1]])

H   = np.matrix([                    [1, 0, 0, 0],
                                     [0, 0, 1, 0]])

P0  = 10*np.eye(4)
Q   = .5*np.matrix(np.eye(4))
R   = 20*np.matrix(np.eye(2))

gamma  = None
gammaW = None

max_covariance = 50
max_velocity = 10

