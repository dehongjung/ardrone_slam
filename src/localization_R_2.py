#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from math import sin, cos, atan2, asin, degrees, radians



from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError


class Localization:

  def __init__(self):
    #inicializar janela para desenho da posicao
    cv2.namedWindow("Localization", 1)
    cv2.namedWindow("O_L", 1)
    self.img = np.empty([1000,1000,3])
    self.img.fill(255)
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)
    self.aux = 1
    self.initial_angle = 0
    self.count=0

    self.target = open("dados.txt", 'w')
    self.target.truncate()

    self.sub_imu = rospy.Subscriber('/ardrone/imu', Imu, self.Rotation_Translation)

    #Inicializacao
    self.dt = 0.0
    self.t = 0.0
    self.aux = 1
    self.q = np.array([[1.0],
                       [0.0],
                       [0.0],
                       [0.0]])
    self.g = np.array([[0.0],
                       [0.0],
                       [-9.8]])
    self.v = np.array([[0.0],
                       [0.0],
                       [0.0]])
    self.p = np.array([[0.0],
                       [0.0],
                       [0.0]])
    #Escala
    for counter in range(0, 1000, 50):
        cv2.line(self.img, (counter,0), (counter,1000), (0,0,0), 1, 4, 0)
        cv2.line(self.img, (1000, counter), (0,counter), (0,0,0), 1, 4, 0)

    cv2.circle(self.img, (500,500), 6, (255,0,0), 3, 8, 0)

  def Rotation_Translation(self, data):

    #=======Tempo de Amostragem=====
    if (self.aux):
      self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
      self.aux = 0
    
    self.dt = (data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)) - self.t
    self.t = data.header.stamp.secs + (data.header.stamp.nsecs/1000000000.0)
    print "NSECS: " ,self.t

    self.target.write(str(self.t))
    self.target.write("\n")
    #self.dt = 0.023
    #============ROTACAO============

    #Velocidade Angular do Gyroscopio 
    wx = data.angular_velocity.x
    wy = data.angular_velocity.y
    wz = data.angular_velocity.z

    #Quaternion
    W = np.array([[0, wx, wy, wz],
                  [-wx, 0, -wz, wy],
                  [-wy, wz, 0, -wx],
                  [-wz, -wy, wx, 0]])
    q_dot = -0.5*np.dot(W,self.q)
    self.q = self.q + self.dt*q_dot

    q0 = self.q[0][0]
    q1 = self.q[1][0]
    q2 = self.q[2][0]
    q3 = self.q[3][0]

    #Matriz de Rotacao
    C = np.array([
      [pow(q0,2)+pow(q1,2)-pow(q2,2)-pow(q3,2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2)],
      [2*(q1*q2-q0*q3), pow(q0,2)-pow(q1,2)+pow(q2,2)-pow(q3,2), 2*(q2*q3+q0*q1)],
      [2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), pow(q0,2)-pow(q1,2)-pow(q2,2)+pow(q3,2)]
      ])

    #Euler
    roll = atan2(2*(q0*q1 +q2*q3), 1-2*(pow(q1,2) + pow(q2,2)))
    pitch = asin(2*(q0*q2 - q3*q1))
    yaw = atan2(2*(q0*q3 +q2*q1), 1-2*(pow(q2,2) + pow(q3,2)))
    if(self.aux):
      self.initial_angle = yaw
      self.aux = 0

    theta_new = degrees(yaw) - self.initial_angle
    #============TRANSLACAO============

    #Forca especifica do Acelerometro
    f = np.array([[data.linear_acceleration.x],
                  [data.linear_acceleration.y],
                  [data.linear_acceleration.z]])
    print "Forca especifica: \t", f

    #Velocidade
    v_dot = (np.dot(C,f) + self.g)
    self.v = self.v + self.dt*v_dot

    #Posicao
    p_dot = self.v
    self.p = self.p + self.dt*self.v    # + 0.5*v_dot*(self.dt*self.dt)
    
    self.Print(v_dot, q_dot, W, roll, pitch, yaw, self.dt)

    cor = 255
    #desenhar posicao estimada (ponto vermelho = takeoff)
    posx=500+self.p[1][0]
    posy=500+self.p[0][0]
    self.img [posx,posy]=(0,0,cor)
    self.img [posx,(posy+1)]=(0,0,cor)
    self.img [posx,(posy-1)]=(0,0,cor)
    self.img [(posx-1),posy]=(0,0,cor)
    self.img [(posx+1),posy]=(0,0,cor)

    #desenho orientacao
    self.img2 = np.empty([100,100,3])
    self.img2.fill(255)

    cv2.line(self.img2, (50,50), (50-int(20*sin(radians(theta_new))), 50-int(20*cos(radians(theta_new)))), (0,0,255), 2, 8, 0)
    cv2.circle(self.img2, (50,50), 1, (255,0,0), 2, 8, 0)  


    #if (self.count > 5000):
    #  self.target.close()

    cv2.imshow("Localization",self.img)
    cv2.imshow("O_L",self.img2)
    cv2.waitKey(1)

  def Print(self, v_dot, q_dot, W, roll, pitch, yaw, dt):
    print "Posicao X: ", self.p[0][0]
    print "Posicao Y: ", self.p[1][0]
    print "Posicao Z: ", self.p[2][0]
    print " "
    print "Velocidade X: ", self.v[0][0]
    print "Velocidade Y: ", self.v[1][0]
    print "Velocidade Z: ", self.v[2][0]
    print " "
    print "Aceleracao X: ", v_dot[0][0]
    print "Aceleracao Y: ", v_dot[1][0]
    print "Aceleracao Z: ", v_dot[2][0]
    self.count=self.count+1;
    print "Count: ", self.count
    #print " "
    #print "q: ", self.q
    #print "q2: ", self.q[0][1]
    #print "q3: ", self.q[0][2]
    #print "q4: ", self.q[0][3]
    #print " "
    #print "W: ", W
#    print "Roll: ", degrees(roll)
#    print "Pitch: ", degrees(pitch)
#    print "Yaw: ", degrees(yaw)
    print "Dt: ", dt
    print "-------------------"


def main(args):
  ic = Localization()
  rospy.init_node('Localization', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)