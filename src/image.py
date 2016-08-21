#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
from matplotlib import pyplot as plt

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from find_obj import filter_matches,explore_match

class image_converter:

  def __init__(self):
    cv2.namedWindow("Image window", 1)
    cv2.namedWindow("ORB Feature", 1)
    #cv2.namedWindow("orb Feature - Without nonmaxSuppression", 1)
    self.count = 1 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.callback)
    #self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    if(self.count):
      # Initiate orb object with default values
      orb = cv2.ORB()

      # find and draw the keypoints
      self.kp1 = orb.detect(cv_image,None)
      self.kp1, self.des1 = orb.compute(cv_image, self.kp1)
      self.img1 = cv2.drawKeypoints(cv_image, self.kp1, color=(255,0,0))

      cv2.imshow("Image window", self.img1)
      print "b"
      self.count = 0
    else:
      orb = cv2.ORB()

      kp2 = orb.detect(cv_image,None)
      kp2, des2 = orb.compute(cv_image, kp2)
      img2 = cv2.drawKeypoints(cv_image, kp2, color=(0,255,0))

      bf = cv2.BFMatcher()
      matches = bf.match(self.des1,des2)

      # Sort them in the order of their distance.
      matches = sorted(matches, key = lambda x:x.distance)

      print matches[1].distance
      print matches[1].trainIdx
      print matches[1].queryIdx
      print matches[1].imgIdx

      # Apply ratio test
      #good = []
      #for m,n in matches:
      #  if m.distance < 0.75*n.distance:
      #    good.append([m])

      # cv2.drawMatchesKnn expects list of lists as matches.
      #img3 = cv2.drawMatches(self.img1,self.kp1,img2,kp2,matches[:10],flags=2)

      #cv2.imshow("ORB Feature", img2)

      # Disable nonmaxSuppression
      #orb.setBool('nonmaxSuppression', 0)
      #kp = orb.detect(cv_image, None)
      #img3 = cv2.drawKeypoints(cv_image, kp, color=(255,0,0))
      
      #cv2.imshow("orb Feature - Without nonmaxSuppression", img3)

      cv2.waitKey(3)


    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError, e:
    #  print e

#def ReceiveData(data):
  #print 'a'
  #print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
  #print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
  #print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)