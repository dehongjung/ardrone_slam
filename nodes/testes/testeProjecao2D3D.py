#!/usr/bin/env python

'''
    Universidade de Brasilia 2016
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Programa para visualizacao da imagem da Camera do Ar.Drone 2.0
'''

# =============== Bibliotecas ===================

import roslib
roslib.load_manifest('ardrone_slam')

import sys
import rospy
import cv2
import tf
import numpy as np
from slam_utils import transf, matrix
from math import sin, cos, radians, sqrt, degrees

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Dimensao da Imagem
x_max = 598
y_max = 292

# ================ Classe ======================

class Camera:

  def __init__(self):

    # Subscriber
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.imageCallback)

    # CvBridge
    self.bridge = CvBridge()

    # Calibracao
    self.mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
    self.dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])
    self.mat_ret = np.array([[449.616018, 0.0, 282.673315], [0.0, 450.589432, 131.813266], [0.0, 0.0, 1.000000]])

  

  # ================= LOOP ===================

  def imageCallback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e 
    
    # Calibracao
    # Imagem Calibrada: 598x292 (X vs Y)
    cv_image = self.Calibrate(cv_image)

    # Initiate ORB Detection
    orb2 = cv2.ORB(1)

    # ORB Keypoints
    kp = orb2.detect(cv_image,None)

    # ORB Descriptors
    kp, des2 = orb2.compute(cv_image, kp)

    # Show Image
    # img = cv2.drawKeypoints(cv_image,kp,color=(0,255,0), flags=0)

    if len(kp) > 0:
      u = kp[0].pt[0]
      v = kp[0].pt[1]

      pos3D_f = transf.Position3D(float(0.72), float(0.07), float(0.0))

      h = self.Project_3D2D (pos3D_f) 

      # cv2.circle(cv_image,(int(h[0]),int(h[1])),3,(0, 0, 250),3,8,0)
      cv2.circle(cv_image,(int(u),int(v)),3,(0, 255, 0),2,8,0)

      pos3D_proj = self.Bootstrap(u, v, 1.5, 100)

      print "--------------"
      print "u: ",u
      print "v: ",v
      print "h[0]: ",h[0]
      print "h[1]: ",h[1]
      print "Projecao 3D: ", pos3D_proj

      cv2.imshow("ORB", cv_image)
      cv2.waitKey(1)


  # ================= FUNCOES ===================

  ## Funcao de Calibracao da Imagem da Camera
  def Calibrate(self, cv_image):
    width = 640
    height = 360
    #calibration
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mat, self.dis, (width, height),1,(width, height))
    # undistort
    dst = cv2.undistort(cv_image, self.mat, self.dis, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    cv_image = dst[y:y+h, x:x+w]
    return cv_image

  ## Funcao que desenha uma elipse da imagem
  def Ellipse (self, x, y, sx, sy, ro, c, img, color):
    alpha = 0.5*np.arctan2(2.0*ro*sx*sy,((sx*sx)-(sy*sy)))
    p1 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*cos(alpha)*sy*cos(alpha))-2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*sin(alpha)*sx*sin(alpha))))
    p2 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*sin(alpha)*sy*sin(alpha))+2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*cos(alpha)*sx*cos(alpha))))
    cv2.ellipse(img,(int(x),int(y)),(int(p1),int(p2)),degrees(alpha),0,360,color,1)

    return img

    ## Funcao realiza Projecao 2D (u,v) a partir da Posicao 3D de uma Feature
  def Project_3D2D (self, pos_w):

    # Transformacao de coordenadas do robo para a camera
    # R_rw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    # P_rw = transf.Position3D (0.0, 0.0, 0.0)
    # H_rw = transf.HomogeneousTransformation3D (R_rw, P_rw)

    # p_cr = np.array([[0.2],[0.0],[0.0],[1.0]])
    # p_cw = np.dot(H_rw, p_cr)

    #Global to relative position transformation
    R_cw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    # P_cw = transf.Position3D (p_cw[0][0], p_cw[1][0], p_cw[2][0])
    P_cw = transf.Position3D (0.0, 0.0, 0.0)
    iH_cw = transf.InverseHomogeneousTransformation3D (R_cw, P_cw)

    R_pw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    P_pw = transf.Position3D (pos_w[0], pos_w[1], pos_w[2])
    H_pw = transf.HomogeneousTransformation3D (R_pw, P_pw)

    H_pc = np.dot(iH_cw, H_pw)

    relativePos = np.transpose(np.array([(-H_pc[1][3]) , -(H_pc[2][3]) , (H_pc[0][3]) ]))   # metros

    # Position projected in the camera 
    #pos_2d = np.dot( self.mat_ret, relativePos )
    u = float(self.mat_ret[0][2]) + (float(self.mat_ret[0][0])*float(relativePos[0]))/float(relativePos[2])
    v = float(self.mat_ret[1][2]) + (float(self.mat_ret[1][1])*float(relativePos[1]))/float(relativePos[2])

    if relativePos[2] < 0:
      u = -999.0

    h = np.array([u, v])

    return h

  ## Funcao realiza Projecao 3D Global a partir de uma Posicao 2D (u,v em pixels) de uma Feature e uma Posicao z
  def Project_2D3D (self, u, v, z):
    #inv_M = matrix.inverseMatrix2(self.mat_ret)
    #aux = np.array([[u],[v],[1.0]])
    #pos3D_rel = z*(np.dot(inv_M, aux))     # metros

    x_rel = z*((u-self.mat_ret[0][2])/self.mat_ret[0][0])
    y_rel = z*((v-self.mat_ret[1][2])/self.mat_ret[1][1])

    # Transformacao de coordenadas do robo para a camera
    R_rw = transf.RotationMatrix3D (0.0,0.0,0.0)
    P_rw = transf.Position3D (0.0,0.0,0.0)
    H_rw = transf.HomogeneousTransformation3D (R_rw, P_rw)

    p_cr = np.array([[0.2],[0.0],[0.0],[1.0]])
    p_cw = np.dot(H_rw, p_cr)

    #Global position transformation
    R_cw = transf.RotationMatrix3D (0.0,0.0,0.0)
    P_cw = transf.Position3D (p_cw[0][0], p_cw[1][0], p_cw[2][0])     # metros
    H_cw = transf.HomogeneousTransformation3D (R_cw, P_cw)

    R_pc = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    # P_pc = transf.Position3D (z, -y_rel, x_rel)   # metros
    P_pc = transf.Position3D (z, -x_rel, -y_rel)   # metros
    H_pc = transf.HomogeneousTransformation3D (R_pc, P_pc)

    H_pw = np.dot(H_cw, H_pc)

    # Posicao Global Feature
    pos3D = np.transpose(np.array([(H_pw[0][3]) , (H_pw[1][3]) , (H_pw[2][3]) ]))      # metros    # MODIFICADO

    return pos3D

  def Bootstrap (self, u, v, z, max_samples):
    u_normal = np.random.normal(u, 1, max_samples)
    v_normal = np.random.normal(v, 1, max_samples)

    passo = float(z)/float(max_samples)

    sum_pos3D = np.transpose(np.array([0.0 , 0.0 , 0.0 ]))

    for i in range(1, (max_samples+1)):
      _z = i*passo
      _pos3D = self.Project_2D3D(u_normal[i-1], v_normal[i-1], _z)

      sum_pos3D += _pos3D

    pos3D = sum_pos3D/float(max_samples)


    # bootstrapSum = np.array([[0.0 , 0.0 , 0.0 ],[0.0 , 0.0 , 0.0 ],[0.0 , 0.0 , 0.0 ]])

    # for i in range(1, (max_samples+1)):
    #   _z = i*passo
    #   _pos3D = self.Project_2D3D(u_normal[i-1], v_normal[i-1], _z)

    #   aux = _pos3D - pos3D
    #   aux = np.array([[ aux[0], aux[1], aux[2] ]])

    #   bootstrapSum += np.dot(np.transpose(aux), aux)
      
    # self.P_bootstr = (bootstrapSum/(float(max_samples)-2.0))
    # self.P           = matrix.insertMatrix(self.P, self.P_bootstr)

    return pos3D


# ============== MAIN ================

def main(args):
  
  rospy.init_node('Camera', anonymous=True)
  ic = Camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)