import numpy as np
from math import cos, sin

def Position3D (x, y, z):
	return np.array([x, y, z])

def Position2D (x, y):
	return np.array([x, y])

def RotationMatrix3D (roll, pitch, yaw):
	R = np.array([
				 [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll)],
				 [sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll)],
				 [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
				 ])
	return R

def RotationMatrix2D (yaw):
	R = np.array([
              	 [cos(yaw), -sin(yaw)], 
              	 [sin(yaw), cos(yaw)]
              	])
	return R

def HomogeneousTransformation3D (R, p):
	H = np.array([
				 [R[0][0], R[0][1], R[0][2], p[0]],
				 [R[1][0], R[1][1], R[1][2], p[1]],
				 [R[2][0], R[2][1], R[2][2], p[2]],
				 [0.0, 0.0, 0.0, 1]
				 ])
	return H

def InverseHomogeneousTransformation3D (R, p):
	tR = np.transpose(R)
	aux = np.dot(-tR, p)
	iH = np.array([
				 [tR[0][0], tR[0][1], tR[0][2], aux[0]],
				 [tR[1][0], tR[1][1], tR[1][2], aux[1]],
				 [tR[2][0], tR[2][1], tR[2][2], aux[2]],
				 [0.0, 0.0, 0.0, 1]
				 ])
	return iH

def InverseHomogeneousTransformation2D (R, p):
	tR = np.transpose(R)
	aux = np.dot(-tR, p)
	iH = np.array([
				 [tR[0][0], tR[0][1], aux[0]],
				 [tR[1][0], tR[1][1], aux[1]],
				 [0.0, 0.0, 0.0, 1]
				 ])
	return iH

def HomogeneousTransformation2D (R, p):
	H = np.array([
                  [R[0][0], R[0][1], p[0]], 
                  [R[1][0], R[1][1], p[1]], 
                  [0.0, 0.0, 1.0]
                  ])
	return H