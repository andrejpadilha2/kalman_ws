from .kalman_filter_nd import KalmanFilterND # class from kalman_filter_nd.py
from .discretization import Q_discrete_white_noise # I don't understand this yet
from scipy.linalg import block_diag
import numpy as np

def pos_vel_filter_in_xy(Q_var, R_var, dt):
	""" Returns a KalmanFilter for position AND velocity on 2 axis
	"""	
	# KALMAN FILTER PARAMETERS
	x = np.array([[0., .2, 0., .2]]).T				# initial system state, position and velocity mean
	P = np.array(	[[121., 0., 0.,  0.],					# initial system state covariance matrix, pos_var = (11m)^2 and vel. var = (5m)^2 and covariances zero
			 [0.,  25., 0.,  0.],
			 [0.,  0., 121., 0.],
			 [0.,  0., 0.,  25.]])
	F = np.array(	[[1., dt, 0.,  0.],				# the state transition matrix (relation of variable states)
			 [0.,  1., 0.,  0.],
			 [0.,  0., 1., dt],
			 [0.,  0., 0.,  1.]])
	q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)	# process covariance matrix
	Q = block_diag(q, q)
	H = np.array([[1., 0., 0., 0.],				# observation matrix, converts from state space to measurement space
		      [0., 0., 1., 0.]])				
	R = np.array([[R_var,     0.],
		      [0.    , R_var]])				# measurement covariance matrix (the variances of each sensor and the covariances between them)
	
	kf = KalmanFilterND(dim_x=4, dim_z=2)
	kf.x = x 
	kf.P = P
	kf.F = F
	kf.Q = Q
	kf.H = H
	kf.R = R
	
	return kf
