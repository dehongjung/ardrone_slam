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

class LeituraDados:

  def __init__(self):
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)



def ReceiveData(data):
	print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
	print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
	print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)
	print '[{0:.3f}] Vel X: {1:.3f}'.format(data.header.stamp.to_sec(), data.vx)
	print '[{0:.3f}] Vel Y: {1:.3f}'.format(data.header.stamp.to_sec(), data.vy)
	print '[{0:.3f}] Vel Z: {1:.3f}'.format(data.header.stamp.to_sec(), data.vz)
	print '[{0:.3f}] Altitude: {1:.3f}'.format(data.header.stamp.to_sec(), data.altd)
	print '[{0:.3f}] Tempo: {1:.3f}'.format(data.header.stamp.to_sec(), data.tm/1000000)

def main(args):
  ic = LeituraDados()
  rospy.init_node('LeituraDados', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)