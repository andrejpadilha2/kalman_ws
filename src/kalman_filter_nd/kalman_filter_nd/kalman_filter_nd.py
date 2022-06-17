import numpy as np
from numpy import dot, zeros, eye
from scipy.linalg import inv

class KalmanFilterND(object):
	""" Defines the Kalman Filter class for n dimensions
	"""

	def __init__(self, dim_x, dim_z):
		self.dim_x = dim_x
		self.dim_z = dim_z
		
		self.x = zeros((dim_x, 1))  	# state variables vector
		self.P = eye(dim_x)		# state covariance matrix
		self.F = eye(dim_x)		# state transition matrix (process model)
		self.Q = eye(dim_x)		# process covariance matrix (process uncertainty)
		self.H = zeros((dim_z, dim_x))	# observation matrix
		self.R = eye(dim_z) 		# sensor/measurement covariance matrix
		
		self.K = zeros((dim_x, dim_z))	# Kalman gain
		self.y = zeros((dim_z, 1))	# residual
		
	def predict(self):
		self.x = dot(self.F, self.x)				# priori
		self.P = dot(dot(self.F, self.P), self.F.T) + self.Q	# priori variance
		
	def update(self, z):  
		S = dot(dot(self.H, self.P), self.H.T) + self.R
		self.K = dot(dot(self.P, self.H.T), inv(S))
		self.y = z - dot(self.H, self.x)
		self.x = self.x + dot(self.K, self.y)      		# posterior
		self.P = self.P - dot(dot(self.K, self.H), self.P)	# posterior variance
