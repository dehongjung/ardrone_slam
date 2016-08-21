import roslib
import rospy
import sys
import cv2

roslib.load_manifest('ardrone_tutorials')

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def __init__ (self):
	cv2.namedWindow("Image Window", 1)
	self.bridge = CvBridge()
	self.sub_Image = rospy.Subscriber('/ardrone/front/image_raw', Image, self.ReceiveImage)

def ReceiveData(data):
  print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)

def ReceiveImage(self, image):
  try:
  	cv_image = self.bridge.imgmsg_to_cv2(image.data, "bgr8")
  except CvBridgeError, e:
  	print e

  cv2.imshow("Image Window", cv_image)
  cv2.waitKey(3)



rospy.init_node('minimum_viable_subscriber')
#sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)
self.sub_Image = rospy.Subscriber('/ardrone/front/image_raw', Image, self.ReceiveImage)

while not rospy.is_shutdown():
  pass