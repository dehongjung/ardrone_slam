#!/usr/bin/env python
import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import numpy as np
from matplotlib import pyplot as plt
from math import sin, cos, atan2, asin, degrees


from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Imu


class Localization:

  def __init__(self):

    self.sub_imu = rospy.Subscriber('/ardrone/imu', Imu, self.Rotation_Translation)

    #Inicializacao
    self.dt = 0.02
    self.q = np.array([[1.0],
                       [0.0],
                       [0.0],
                       [0.0]])
    self.g = np.array([[-0.00055],
                       [0.009717],
                       [9.302286]])
    self.v = np.array([[0.0],
                       [0.0],
                       [0.0]])
    self.p = np.array([[0.0],
                       [0.0],
                       [0.0]])


  def Rotation_Translation(self, data):
    #============ROTACAO============

    #Velocidade Angular do Gyroscopio 
    wx = data.angular_velocity.x
    wy = data.angular_velocity.y
    wz = data.angular_velocity.z

    #Quaternion
    W = np.array([[0, -wx, wy, wz],
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


    #============TRANSLACAO============

    #Forca especifica do Acelerometro
    f = np.array([[data.linear_acceleration.x],
                  [data.linear_acceleration.y],
                  [data.linear_acceleration.z]])

    #Velocidade
    v_dot = C*f #+ self.g
    self.v = self.v + self.dt*v_dot

    #Posicao
    p_dot = self.v
    self.p = self.p + self.dt*self.v
    
    self.Print(v_dot, q_dot, W, roll, pitch, yaw)


  def Print(self, v_dot, q_dot, W, roll, pitch, yaw):
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
    #print " "
    #print "q: ", self.q
    #print "q2: ", self.q[0][1]
    #print "q3: ", self.q[0][2]
    #print "q4: ", self.q[0][3]
    #print " "
    #print "W: ", W
    print "Roll: ", degrees(roll)
    print "Pitch: ", degrees(pitch)
    print "Yaw: ", degrees(yaw)
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