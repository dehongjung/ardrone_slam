#!/usr/bin/env python

'''
	Universidade de Brasilia 2016
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Plotagem da Localizacao 2D e Orientacao
'''

# =============== Bibliotecas ===================

import roslib
roslib.load_manifest('ardrone_slam')

import sys
import rospy
import cv2
import tf
import numpy as np
import message_filters
from slam_utils import transf

from math import sin, cos, radians, sqrt, degrees

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
#from find_obj import filter_matches,explore_match
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Dimensao da Imagem
x_max = 598
y_max = 292

# ================ Classe ======================


class Plot:

  def __init__(self):

    #########################
    ##     Localization    ##
    #########################

    # Janelas
    #cv2.namedWindow("Pose", 1)
    #cv2.namedWindow("O_P", 1)

    # Subscriber
    self.sub = rospy.Subscriber('pose3D_plot', Pose, self.Plot)

    # Imagens
    self.img = np.empty([1000,1000,3])
    self.img.fill(255)
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    # Variaveis
    self.counter = 0

    # Escala
    for counter in range(0, 1000, 30):
        cv2.line(self.img, (counter,0), (counter,1000), (0,0,0), 1, 4, 0)
        cv2.line(self.img, (1000, counter), (0,counter), (0,0,0), 1, 4, 0)


    cv2.circle(self.img, (500,500), 6, (255,0,0), 3, 8, 0)



  # ================= LOOP ===================

  def Plot(self, pose):

    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    cor = self.Scale(100, 50000, 0, 255, pose.position.z)

    # Desenho da Posicao X e Y estimada (ponto vermelho = takeoff)
    if pose.position.z != 0:
      posx=500-pose.position.x/10
      posy=500-pose.position.y/10 
      self.img [posx,posy]=(0 , 0, cor)
      self.img [posx,(posy+1)]=(0, 0, cor)
      self.img [posx,(posy-1)]=(0, 0, cor)
      self.img [(posx-1),posy]=(0, 0, cor)
      self.img [(posx+1),posy]=(0, 0, cor)

    cv2.circle(self.img, (500,500-240), 1, (255,0,0), 2, 8, 0)

    # Desenho da Orientacao
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    cv2.line(self.img2, (50,50), (50-int(20*sin(yaw)), 50-int(20*cos(yaw))), (0,0,255), 2, 8, 0)
    cv2.circle(self.img2, (50,50), 1, (255,0,0), 2, 8, 0)   

    # Janelas
    cv2.imshow("Pose1",self.img)
    cv2.imshow("O_P",self.img2)
    cv2.waitKey(1)


  # ================= FUNCOES ===================
  def Scale (self, old_min, old_max, new_min, new_max, old_value):
    if old_value > old_max:
        new_value = new_max

    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min)*new_range)/old_range) + new_min
    return new_value


# ============== MAIN ================

def main(args):
  ic = Plot()
  rospy.init_node('Plot', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
