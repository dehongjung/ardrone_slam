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

class Calibration:

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