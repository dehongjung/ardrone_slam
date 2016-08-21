#!/usr/bin/env python

'''
	Universidade de Brasilia
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Deteccao de Features usando algoritmo ORB no Quadrotor Parrot 2.0
'''

# =============== Bibliotecas ===================

import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from find_obj import filter_matches,explore_match

# ================ Classe ======================

class ORB:

  def __init__(self):
    #cv2.namedWindow("ORB", 1)

    #cv2.namedWindow("ORB Feature", 1)

    self.count = 1 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.callback)
    # self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.callback)

    # Publisher
    self.pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

    #self.img_inicial = cv2.imread('imagem.png',1)
    #cv2.imshow("ORB", self.img_inicial) 
    # Initiate STAR detector
    #self.orb1 = cv2.ORB(40)

    # find the keypoints with ORB
    #self.kp1 = self.orb1.detect(self.img_inicial,None)

    # compute the descriptors with ORB
    #self.kp1, self.des1 = self.orb1.compute(self.img_inicial, self.kp1)


  def Calibrate(self, cv_image):
    self.mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
    self.dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])
    self.w = 640
    self.h = 360
    #calibration
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mat, self.dis, (self.w,self.h),1,(self.w,self.h))
    # undistort
    dst = cv2.undistort(cv_image, self.mat, self.dis, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    cv_image = dst[y:y+h, x:x+w]
    return cv_image


  # ================= LOOP ===================

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e 

    cv_image = self.Calibrate(cv_image)

    calib_img = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    self.pub.publish(calib_img)

    # Initiate STAR detector
    orb2 = cv2.ORB(3)
    # find the keypoints with ORB
    kp2 = orb2.detect(cv_image,None)

    # compute the descriptors with ORB
    kp2, des2 = orb2.compute(cv_image, kp2)

    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(cv_image,kp2,color=(0,255,0), flags=0)

    #img2 = np.zeros((360,640,3), np.uint8)
    cv2.imshow("ORB", img2)

    # cv2.imshow("ORB", self.img_inicial) 

    # BFMatcher with default params
    # bf = cv2.BFMatcher()
    # matches = bf.knnMatch(self.des1,des2, k=2)

    # # Apply ratio test
    # good = []
    # count = 0
    # for m,n in matches:
    #   if m.distance < 0.8*n.distance:
    #     count = count + 1
    #     good.append([m])

    # print "Numero de Matches: ", count
    # print "Numero de Features Atual: ", len(des2)
    # print "Numero de Features Inicial: ", len(self.des1)
    # print "--------------------"

    cv2.waitKey(3)


# ============== MAIN ================

def main(args):
  ic = ORB()
  rospy.init_node('ORB', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)