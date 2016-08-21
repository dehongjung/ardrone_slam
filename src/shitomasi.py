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

class Shi_Tomasi:

  def __init__(self):
    cv2.namedWindow("Image window", 1)

    self.count = 1 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.callback)
    #self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

    corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
    corners = np.int0(corners)

    for i in corners:
      x,y = i.ravel()
      cv2.circle(cv_image,(x,y),3,255,-1)

    cv2.imshow("Image window", cv_image)


    cv2.waitKey(3)


    try:
     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
     print e

#def ReceiveData(data):
  #print 'a'
  #print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
  #print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
  #print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)

def main(args):
  ic = Shi_Tomasi()
  rospy.init_node('Shi_Tomasi', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)