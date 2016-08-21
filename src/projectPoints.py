import cv2
import numpy as np
import glob
from math import sin, cos, radians


mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])
w = 640
h = 360

axis = np.float32([[0,0.2,0]]).reshape(-1,3)  # Posicao da feature em relacao ao robo
rvecs  = np.float32([[0,0,0]])
tvecs  = np.float32([[0,0,0]])

# project 3D points to image plane
imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mat, dis)

print imgpts
