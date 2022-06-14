from .kalman_filter_1d import KalmanFilter1D # class from kalman_filter_1d.py

def position_filter_1d(x, P, vel, R, Q=0., dt=1.0):
	""" Returns a KalmanFilter in 1D (only for position)
	"""	
	kf = KalmanFilter1D()
	kf.R = R		# sensor/measurement variance
	kf.x = x 		# initial position mean
	kf.P = P 		# initial position variance
	kf.dx = vel*dt		# process model
	kf.Q = Q		# process variance
	
	return kf
