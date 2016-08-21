#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
from matplotlib import pyplot as plt
import numpy as np
from math import sin, cos, radians

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from find_obj import filter_matches,explore_match


class Pose3D:

  def __init__(self):
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.callback)


    self.position = np.array([[0], [0]])
    self.old_angle = 0
    self.old_x = 0
    self.old_y = 0
    self.old_t = 0.0
    self.old_vx = 0
    self.old_vy = 0

  def callback(self, data):
    '''
    :param t: time since simulation start
    :param dt: time since last call to measurement_callback
    :param navdata: measurements of the quadrotor
    '''
    
    # TODO: update self.position by integrating measurements contained in navdata
    new_t = data.tm/1000000
    delta_t2 = new_t - self.old_t
    delta_t = 0.02
    #theta_new = data.rotZ - self.old_angle

    x = data.vx*delta_t + 0.5*data.ax*(delta_t*delta_t)
    y = data.vy*delta_t + 0.5*data.ay*(delta_t*delta_t)
    
    T_old = np.array([[cos(self.old_angle), -sin(self.old_angle), self.old_x], [sin(self.old_angle), cos(self.old_angle), self.old_y], [0,0,1]])
    T_new = np.array([[cos(radians(data.rotZ)), -sin(radians(data.rotZ)), x], [sin(radians(data.rotZ)), cos(radians(data.rotZ)), y], [0,0,1]])
    T = np.dot(T_old, T_new)
    
    self.position[0] = T[0][2]
    self.position[1] = T[1][2]
    
    #print x," ", y
    print 'Posicao X: ', self.position[0]
    print 'Posicao Y: ', self.position[1]
    print 'Posicao Z: ', data.altd
    print 'Angulo Z: ', data.rotZ
    print 'Velocidade X: ', data.vx
    print 'Velocidade Y: ', data.vy
    print 'Tempo Velho: ', new_t
    print 'Tempo  Novo: ', self.old_t
    print 'Tempo: ', delta_t2

    print '---------------------------'


    
    self.old_t = new_t
    self.old_x = self.position[0]
    self.old_y = self.position[1]
    self.old_angle = radians(data.rotZ)


  def ReceiveData(data):
  	print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
  	print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
  	print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)
  	print '[{0:.3f}] Vel X: {1:.3f}'.format(data.header.stamp.to_sec(), data.vx)
  	print '[{0:.3f}] Vel Y: {1:.3f}'.format(data.header.stamp.to_sec(), data.vy)
  	print '[{0:.3f}] Vel Z: {1:.3f}'.format(data.header.stamp.to_sec(), data.vz)
  	print '[{0:.3f}] Altitude: {1:.3f}'.format(data.header.stamp.to_sec(), data.altd)


def main(args):
  ic = Pose3D()
  rospy.init_node('Pose3D', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)