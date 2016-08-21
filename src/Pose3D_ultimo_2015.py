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


class Pose3D:

  def __init__(self):
    #inicializar janela para desenho da posicao
    cv2.namedWindow("Pose", 1)
    cv2.namedWindow("O_P", 1)

    self.img = np.empty([1000,1000,3])
    self.img.fill(255)
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.callback)

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

    self.target = open("dados.txt", 'w')
    self.target.truncate()

    #Escala
    for counter in range(0, 1000, 50):
        cv2.line(self.img, (counter,0), (counter,1000), (0,0,0), 1, 4, 0)
        cv2.line(self.img, (1000, counter), (0,counter), (0,0,0), 1, 4, 0)



    cv2.circle(self.img, (500,500), 6, (255,0,0), 3, 8, 0)


  def callback(self, data):
    '''
    :param t: time since simulation start
    :param dt: time since last call to measurement_callback
    :param navdata: measurements of the quadrotor
    '''
    #=======Tempo de Amostragem=====
    if (self.aux2):
      self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
      self.aux2 = 0
    
    self.dt = (data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)) - self.t
    self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
    delta_t = self.dt

   
    #self.dt = 0.023

    if(self.aux):
        self.initial_angle = data.rotZ
        self.aux = 0
    

    # TODO: update self.position by integrating measurements contained in navdata
    #delta_t = 0.02
    theta_new = data.rotZ - self.initial_angle

    x = (data.vx)*delta_t + 0.5*data.ax*9806*(delta_t*delta_t)
    y = (data.vy)*delta_t + 0.5*data.ay*9806*(delta_t*delta_t)
    
    T_old = np.array([[cos(self.old_angle), -sin(self.old_angle), self.old_x], [sin(self.old_angle), cos(self.old_angle), self.old_y], [0,0,1]])
    T_new = np.array([[cos(radians(theta_new)), -sin(radians(theta_new)), x], [sin(radians(theta_new)), cos(radians(theta_new)), y], [0,0,1]])
    T = np.dot(T_old, T_new)
    
    self.position[0] = T[0][2]
    self.position[1] = T[1][2]
    
    #====== Escreve dados em arquivo =============
    # dt \t x \t y \t z
    self.target.write(str(self.t))
    self.target.write("\t")
    self.target.write(str(self.position[0]))
    self.target.write("\t")
    self.target.write(str(self.position[1]))
    self.target.write("\t")
    self.target.write(str(data.altd))
    self.target.write("\n")

    print 'Posicao X: ', self.position[0]
    print 'Posicao Y: ', self.position[1]
    print 'Posicao Z: ', data.altd
    print 'Angulo Z: ', theta_new
    print 'Velocidade X: ', data.vx
    print 'Velocidade Y: ', data.vy
    print 'Angulo Inicial: ', self.initial_angle


    cor = self.Scale(100, 50000, 0, 255, data.altd)
    #desenhar posicao estimada (ponto vermelho = takeoff)
    if data.altd != 0:
        posx=500-self.position[0]/10
        posy=500-self.position[1]/10 
        self.img [posx,posy]=(0 , 0, cor)
        self.img [posx,(posy+1)]=(0, 0, cor)
        self.img [posx,(posy-1)]=(0, 0, cor)
        self.img [(posx-1),posy]=(0, 0, cor)
        self.img [(posx+1),posy]=(0, 0, cor)

    #desenho orientacao
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    cv2.line(self.img2, (50,50), (50-int(20*sin(radians(theta_new))), 50-int(20*cos(radians(theta_new)))), (0,0,255), 2, 8, 0)
    cv2.circle(self.img2, (50,50), 1, (255,0,0), 2, 8, 0)  

    self.count = self.count + 1
    #if (self.count > 1500):
    #  self.target.close()

    cv2.imshow("Pose",self.img)
    cv2.imshow("O_P",self.img2)
    cv2.waitKey(1)

    print '---------------------------'
    

    self.old_x = self.position[0]
    self.old_y = self.position[1]
    self.old_angle = radians(theta_new)


  def ReceiveData(data):
  	print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)
  	print '[{0:.3f}] Roll: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotX)
  	print '[{0:.3f}] Yaw: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotZ)
  	print '[{0:.3f}] Vel X: {1:.3f}'.format(data.header.stamp.to_sec(), data.vx)
  	print '[{0:.3f}] Vel Y: {1:.3f}'.format(data.header.stamp.to_sec(), data.vy)
  	print '[{0:.3f}] Vel Z: {1:.3f}'.format(data.header.stamp.to_sec(), data.vz)
  	print '[{0:.3f}] Altitude: {1:.3f}'.format(data.header.stamp.to_sec(), data.altd)
  
  def Scale(self, old_min, old_max, new_min, new_max, old_value):
    if old_value > old_max:
        new_value = new_max

    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min)*new_range)/old_range) + new_min
    return new_value


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
