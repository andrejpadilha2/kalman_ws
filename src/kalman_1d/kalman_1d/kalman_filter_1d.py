class KalmanFilter1D(object):
	""" Defines the Kalman Filter class for 1 dimension
	"""

	def __init__(self):
		self.R = 1. 	# sensor/measurement variance

		self.x = 0.  	# initial state mean
		self.P = 1.	# initial state variance
		
		self.dx = 1.	# process model
		self.Q = 1.	# process variance
		
		self.K = 0.5	# Kalman gain
		self.y = 1.	# residual
		
	def predict(self):
		self.x = self.x + self.dx	# priori
		self.P = self.P + self.Q	# priori variance
		
	def update(self, z):  
		self.y = z - self.x
		
		self.K = self.P / (self.P + self.R)

		self.x = self.x + self.K*self.y      # posterior
		self.P = (1 - self.K) * self.P  # posterior variance
