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
    img = cv2.drawKeypoints(cv_image,kp,color=(0,255,0), flags=0)
    cv2.imshow("ORB", img)
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