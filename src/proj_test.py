import numpy as np 
from robotics import transf
from math import radians
import cv2


def Project_3D2D (pos_fw, pos_cw):

	#Global to relative position transformation
	R_cw = transf.RotationMatrix3D (roll, pitch, yaw)
	P_cw = transf.Position3D (pos_cw[0], pos_cw[1], pos_cw[2])
	iH_cw = transf.InverseHomogeneousTransformation3D (R_cw, P_cw)

	R_pw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
	P_pw = transf.Position3D (pos_fw[0], pos_fw[1], pos_fw[2])
	H_pw = transf.HomogeneousTransformation3D (R_pw, P_pw)

	H_pc = np.dot(iH_cw, H_pw)

	relativePos = np.transpose(np.array([(H_pc[0][3])/1000.0 , -(H_pc[2][3])/1000.0 , (H_pc[1][3])/1000.0 ])) 

	#Position projected in the camera 
	pos_2d = np.dot( mat_ret, relativePos )

	print relativePos

	return pos_2d


roll = radians(0.0)
pitch = radians(0.0)
yaw = radians(0.0)

x = 0.0
y = 0.0
z = 0.0

R_cw = transf.RotationMatrix3D (roll, pitch, yaw)
P_cw = transf.Position3D (x, y, z)
iH_cw = transf.InverseHomogeneousTransformation3D (R_cw, P_cw)

R_pw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
P_pw = transf.Position3D (0.0, 2400.0, 1000.0)
H_pw = transf.HomogeneousTransformation3D (R_pw, P_pw)

H_pc = np.dot(iH_cw, H_pw)

P_pc = np.array([H_pc[0][3], H_pc[1][3], H_pc[2][3]])

axis = np.float32([[(P_pc[0])/1000.0 , (P_pc[2])/1000.0 , (P_pc[1])/1000.0 ]]).reshape(-1,3)

print axis

rvecs  = np.float32([[0,0,0]])
tvecs  = np.float32([[0,0,0]])

mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])

mat_ret = np.array([[449.616018, 0.0, 282.673315], [0.0, 450.589432, 131.813266], [0.0, 0.0, 1.000000]])

# imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mat_ret, dis)

# print imgpts

relativePos = np.transpose(np.array([(H_pc[0][3])/1000.0 , -(H_pc[2][3])/1000.0 , (H_pc[1][3])/1000.0 ])) 

#Position projected in the camera 
pos_2d = np.dot( mat_ret, relativePos )

# print pos_2d

# print "---------------"

pos = transf.Position3D(0.0, 0.0, 0.0)
pos_f = transf.Position3D(0.0, 2400.0, -1000.0)

print Project_3D2D(pos_f, pos)


