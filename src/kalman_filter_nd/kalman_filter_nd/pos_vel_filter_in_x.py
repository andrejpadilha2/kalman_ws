from .kalman_filter_nd import KalmanFilterND # class from kalman_filter_nd.py
from .discretization import Q_discrete_white_noise # I don't understand this yet
import numpy as np

def pos_vel_filter_in_x(Q_var, R_var, dt):
	""" Returns a KalmanFilter for position AND velocity on 1 axis
	"""	
	# KALMAN FILTER PARAMETERS
	x = np.array([[0., .2]]).T				# initial system state, position and velocity mean
	P = np.diag([121., 25.])					# initial system state covariance matrix, pos_var = (11m)^2 and vel. var = (5m)^2 and covariances zero
	F = np.array(	[[1., dt],				# the state transition matrix (relation of variable states)
			 [0.,  1.]])
	Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)	# process covariance matrix
	H = np.array([[1., 0.]])				# observation matrix, converts from state space to measurement space
	R = np.array([[R_var]])				# measurement covariance matrix (the variances of each sensor and the covariances between them)
	
	kf = KalmanFilterND(dim_x=2, dim_z=1)
	kf.x = x 
	kf.P = P
	kf.F = F
	kf.Q = Q
	kf.H = H
	kf.R = R
	
	return kf
