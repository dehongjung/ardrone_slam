#!/usr/bin/env python

"""
    Universidade de Brasilia
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Localizacao 3D do Quadrotor Parrot 2.0

    Obs: Considerou-se a posicao Z global como o valor lido diretamente do sensor ultra-sonico
"""


# =========== Bibliotecas ================

import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
import tf
from matplotlib import pyplot as plt
import numpy as np
from math import sin, cos, radians

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


# ============== Classe ===================

class Pose3D:

  # ================ SETUP ====================

  def __init__(self):
    
    # Janelas
    cv2.namedWindow("Pose", 1)
    cv2.namedWindow("O_P", 1)

    # Subscriber
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.callback)

    # Publisher
    self.pub = rospy.Publisher('pose3D', MarkerArray)
    self.markerArray = MarkerArray()
    self.pub_features = rospy.Publisher('features', MarkerArray)
    self.markerArray_features = MarkerArray()

    # Imagens
    self.img = np.empty([1000,1000,3])
    self.img.fill(255)
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    # Variaveis
    self.position = np.array([[0], [0]])
    self.old_angle = 0
    self.old_x = 0
    self.old_y = 0
    self.old_vx = 0
    self.old_vy = 0
    self.initial_angle = 0
    self.aux = 1
    self.aux2 = 1
    self.t = 0.0
    self.dt = 0.0
    self.count = 0
    self.counter = 0
    self.counter_to_publish = 0
    self.MARKERS_MAX = 5000

    # Arquivos
    self.target = open("dados.txt", 'w')
    self.target.truncate()

    # Escala
    for counter in range(0, 1000, 50):
        cv2.line(self.img, (counter,0), (counter,1000), (0,0,0), 1, 4, 0)
        cv2.line(self.img, (1000, counter), (0,counter), (0,0,0), 1, 4, 0)


    cv2.circle(self.img, (500,500), 6, (255,0,0), 3, 8, 0)


  # ============= LOOP ================

  def callback (self, data):
    '''
    :param t: time since simulation start
    :param dt: time since last call to measurement_callback
    :param navdata: measurements of the quadrotor
    '''
    ## Tempo de Amostragem
    if (self.aux2):
      self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
      self.aux2 = 0
    
    self.dt = (data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)) - self.t
    self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
    delta_t = self.dt
   
    #self.dt = 0.023

    ## Inicializacao da Orientacao (Yaw)
    if(self.aux):
        self.initial_angle = data.rotZ
        self.aux = 0
    

    #delta_t = 0.02

    ## Definicao do novo theta
    theta_new = data.rotZ - self.initial_angle

    ## Calculo de deslocamento relativo X e Y
    x = (data.vx)*delta_t + 0.5*data.ax*9806*(delta_t*delta_t)
    y = (data.vy)*delta_t + 0.5*data.ay*9806*(delta_t*delta_t)
    
    ## Tranformacao de coordenadas X e Y (Relativa -> Global)
    T_old = np.array([
                      [cos(self.old_angle), -sin(self.old_angle), self.old_x], 
                      [sin(self.old_angle), cos(self.old_angle), self.old_y], 
                      [0,0,1] ])
    T_new = np.array([
                      [cos(radians(theta_new)), -sin(radians(theta_new)), x], 
                      [sin(radians(theta_new)), cos(radians(theta_new)), y], 
                      [0,0,1]])
    T = np.dot(T_old, T_new)
    
    self.position[0] = T[0][2]  # X Global
    self.position[1] = T[1][2]  # Y Global
    
    ## Escrita de Dados de Posicao e Tempo em um arquivo txt
    limit = 2500  # Limitador para parar a escrita
    # self.WriteFile(data.altd, limit)

    ## Print
    print 'Posicao X: ', self.position[0]
    print 'Posicao Y: ', self.position[1]
    print 'Posicao Z: ', data.altd
    print 'Angulo Z: ', theta_new
    print 'Velocidade X: ', data.vx
    print 'Velocidade Y: ', data.vy
    print 'Angulo Inicial: ', self.initial_angle
    print '---------------------------'


    ## Vizualizacao

    cor = self.Scale(100, 50000, 0, 255, data.altd)

    # Desenho da Posicao X e Y estimada (ponto vermelho = takeoff)
    if data.altd != 0:
      posx=500-self.position[0]/10
      posy=500-self.position[1]/10 
      self.img [posx,posy]=(0 , 0, cor)
      self.img [posx,(posy+1)]=(0, 0, cor)
      self.img [posx,(posy-1)]=(0, 0, cor)
      self.img [(posx-1),posy]=(0, 0, cor)
      self.img [(posx+1),posy]=(0, 0, cor)

    # Desenho da Orientacao
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    cv2.line(self.img2, (50,50), (50-int(20*sin(radians(theta_new))), 50-int(20*cos(radians(theta_new)))), (0,0,255), 2, 8, 0)
    cv2.circle(self.img2, (50,50), 1, (255,0,0), 2, 8, 0)  

    # Janelas
    cv2.imshow("Pose",self.img)
    cv2.imshow("O_P",self.img2)
    cv2.waitKey(1)
    
    ## Atualizacao das varaveis de posicao
    self.old_x = self.position[0]
    self.old_y = self.position[1]
    self.old_angle = radians(theta_new)

    ## Quaternion
    quaternion =  tf.transformations.quaternion_from_euler(radians(data.rotX), radians(data.rotY), radians(theta_new))

    ######################
    ## Publisher Pose3D ##
    ######################
    marker = Marker()
    marker.header.frame_id = "/neck"
    marker.type = marker.ARROW

    marker.action = marker.ADD
    marker.scale.x = 40
    marker.scale.y = 10
    marker.scale.z = 6
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.pose.position.x = float(self.position[0])
    marker.pose.position.y = float(self.position[1])
    marker.pose.position.z = float(data.altd)

    marker.id = self.counter

    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
    # if(self.counter > self.MARKERS_MAX):
    #     self.markerArray.markers.pop(1)

    self.markerArray.markers.append(marker)

    # Renumber the marker IDs
    # id = 0
    # for m in self.markerArray.markers:
    #   m.id = id
    #   id += 1

    # Publish the MarkerArray
    # self.pub.publish(self.markerArray)

    ########################
    ## Publisher features ##
    ########################
    marker_feature = Marker()
    marker_feature.header.frame_id = "/neck"
    marker_feature.type = marker_feature.CUBE

    marker_feature.action = marker_feature.ADD
    marker_feature.scale.x = 40
    marker_feature.scale.y = 40
    marker_feature.scale.z = 40
    marker_feature.color.a = 1.0
    marker_feature.color.r = 1.0
    marker_feature.color.g = 0.0
    marker_feature.color.b = 1.0
    marker_feature.pose.orientation.w = 1.0
    marker_feature.pose.position.x = float(self.counter)
    marker_feature.pose.position.y = float(self.counter)
    marker_feature.pose.position.z = data.altd

    marker_feature.id = self.counter
    # if(self.counter > self.MARKERS_MAX):
    #     self.markerArray_features.markers.pop(1)

    self.markerArray_features.markers.append(marker_feature)

    # Publish the MarkerArray
    # self.pub_features.publish(self.markerArray_features)

    if self.counter > 15:       
        self.pub.publish(self.markerArray)
        self.counter_to_publish = 0

    self.counter += 1
    self.counter_to_publish +=1



  # ============== FUNCOES =================

  def ReceiveData (data):
    print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
    print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
    print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)
    print '[{0:.3f}] Vel X: {1:.3f}'.format(data.header.stamp.to_sec(), data.vx)
    print '[{0:.3f}] Vel Y: {1:.3f}'.format(data.header.stamp.to_sec(), data.vy)
    print '[{0:.3f}] Vel Z: {1:.3f}'.format(data.header.stamp.to_sec(), data.vz)
    print '[{0:.3f}] Altitude: {1:.3f}'.format(data.header.stamp.to_sec(), data.altd)
  
  def Scale (self, old_min, old_max, new_min, new_max, old_value):
    if old_value > old_max:
        new_value = new_max

    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min)*new_range)/old_range) + new_min
    return new_value

  def WriteFile (self, altd, limit):
    # dt \t x \t y \t z
    self.target.write(str(self.t))
    self.target.write("\t")
    self.target.write(str(self.position[0]))
    self.target.write("\t")
    self.target.write(str(self.position[1]))
    self.target.write("\t")
    self.target.write(str(altd))
    self.target.write("\n")

    self.count = self.count + 1
    if (self.count > limit):
      self.target.close()


# ================== MAIN ========================

def main(args):
  ic = Pose3D()
  rospy.init_node('Pose3D', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
