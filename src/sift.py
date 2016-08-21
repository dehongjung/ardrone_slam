#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt


from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from find_obj import filter_matches,explore_match

class Fast:

  def __init__(self):
    cv2.namedWindow("Image window", 1)
    #cv2.namedWindow("ORB Feature", 1)

    self.count = 1 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.callback)
    #self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)


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


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e 

    cv_image = self.Calibrate(cv_image)

    # Initiate SIFT detector
    sift = cv2.SIFT(40)

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(cv_image,None)

    img1 = cv2.drawKeypoints(cv_image, kp1, color=(255,0,0))

    cv2.imshow("Image window", img1)


    cv2.waitKey(3)


def main(args):
  ic = Fast()
  rospy.init_node('Fast', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)