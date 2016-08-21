#!/usr/bin/env python

'''
    Universidade de Brasilia 2016
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: SLAM Monocular baseado no EKF usando Quadrirotor AR.Drone Parrot 2.0
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
from slam_utils import transf, matrix

from math import sin, cos, radians, sqrt, degrees

from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Dimensao da Imagem
x_max = 598
y_max = 292

# ================ Classe ======================

class SLAM:

  def __init__(self):

    #########################
    ##       Vision        ##
    #########################

    # Subscriber
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.imageCallback)

    # Publisher
    self.pub_features = rospy.Publisher('features', MarkerArray, queue_size=10)
    # self.pub_image = rospy.Publisher("/camera/image_raw", Image)

    # Markers
    self.markerArray_features = MarkerArray()

    # CvBridge
    self.bridge = CvBridge()

    # Calibracao
    self.mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
    self.dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])
    self.mat_ret = np.array([[449.616018, 0.0, 282.673315], [0.0, 450.589432, 131.813266], [0.0, 0.0, 1.000000]])

    # Variaveis
    self.count = 1 
    self.counter2 = 0

    #########################
    ##     Localization    ##
    #########################

    # Subscriber
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.navdataCallback)

    # Main Loop
    rospy.Timer(rospy.Duration(0.005), self.loop)


    # Publisher
    self.pub_pose = rospy.Publisher('pose3D', MarkerArray, queue_size=10)
    self.pub_pose_plot = rospy.Publisher('pose3D_plot', Pose, queue_size=10)

    # Markers
    self.markerArray = MarkerArray()

    # Inicializacao da posicao e orientacao do robo
    self.pose = Pose()


    # Variaveis

    # Vetor de Estados
    self.X = np.array([[0.0],
                      [0.0],
                      [0.0],
                      [0.0],
                      [0.0],
                      [0.0]]) 

    # Posicao e Orientacao
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0
    self.yaw = 0.0
    self.roll = 0.0
    self.pitch = 0.0

    self.old_angle = 0.0
    self.old_x = 0.0
    self.old_y = 0.0
    self.old_vx = 0.0
    self.old_vy = 0.0
    self.initial_angle = 0.0


    # Matrizes de Projecao e Covariancia
    self.P_imu = np.array([
                          [0.002, 0, 0, 0, 0, 0],
                          [0, 0.002, 0, 0, 0, 0],
                          [0, 0, 0.002, 0, 0, 0],
                          [0, 0, 0, 0.001, 0, 0],
                          [0, 0, 0, 0, 0.001, 0],
                          [0, 0, 0, 0, 0, 0.001]
                          ])
    # self.P_imu = np.zeros((6,6))
    self.P = self.P_imu
    self.P_bootstr = np.zeros((3,3))
    self.Qv = np.array([
                      [0.002, 0, 0, 0, 0, 0],
                      [0, 0.002, 0, 0, 0, 0],
                      [0, 0, 0.002, 0, 0, 0],
                      [0, 0, 0, 0.001, 0, 0],
                      [0, 0, 0, 0, 0.001, 0],
                      [0, 0, 0, 0, 0, 0.001]
                      ])

    self.R = np.array([
                      [0.0001, 0, 0],
                      [0, 0.0001, 0],
                      [0, 0, 0.0001],
                      ])

    self.H = np.array([
                      [0,0,0,1,0,0],
                      [0,0,0,0,1,0],
                      [0,0,0,0,0,1]
                      ])

    # Vetor de Features (Posicao 3D)
    self.featuresArray = np.array([[0.0, 0.0, 0.0]])
    self.featuresArray = np.delete(self.featuresArray, (0), axis=0)

    # Tempo
    self.t = 0.0
    self.dt = 0.0

    # Auxiliares
    self.aux = 1
    self.aux2 = 1
    self.counter = 0
    self.init = 1
    self.count_navdata = 0
    self.count_image = 0
    
    # Variaveis de Sincronizacao
    self.semaphore = 0
    self.sync_counter = 0

    # Listas
    self.controlList = []
    self.navdataList = []
    self.imageList = []
  

  # ================= LOOP ===================

  def navdataCallback(self, navdata):
    self.controlList.append(True)
    self.navdataList.append(navdata)

    # Inicializacao da Orientacao
    if self.count_navdata == 0:
      self.X[3][0] = navdata.rotX
      self.X[4][0] = navdata.rotY
      self.X[5][0] = navdata.rotZ
      self.count_navdata = 1

  def imageCallback(self, data):
    self.controlList.append(False)
    self.imageList.append(data)

  def loop(self, event):

    if self.controlList:

      # Controle
      controlFlag = self.controlList.pop(0)

      # ********** Correcao Navdata ************
      if (controlFlag):

        # ********* Predicao ***********

        navdata = self.navdataList.pop(0)

        ## Tempo de Amostragem
        if (self.aux2):
          self.t = navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)
          self.aux2 = 0
        
        self.dt = (navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)) - self.t
        self.t = navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)
        delta_t = self.dt
       
        #self.dt = 0.023

        ## Atualizacao da Orientacao
        self.yaw = radians(navdata.rotZ)
        self.pitch = radians(navdata.rotY)
        self.roll = radians(navdata.rotX)

        ## Calculo de deslocamento relativo X e Y
        x = float((navdata.vx)*delta_t + 0.5*navdata.ax*9.806*(delta_t*delta_t))/1000.0   #metros
        y = float((navdata.vy)*delta_t + 0.5*navdata.ay*9.806*(delta_t*delta_t))/1000.0
        
        ## Tranformacao de coordenadas X e Y (Relativa -> Global)
        R_old = transf.RotationMatrix2D(self.old_angle)
        p_old = transf.Position2D(self.x, self.y)
        T_old = transf.HomogeneousTransformation2D(R_old, p_old)

        R_new = transf.RotationMatrix2D(self.yaw)
        p_new = transf.Position2D(x, y)
        T_new = transf.HomogeneousTransformation2D(R_new, p_new)

        T = np.dot(T_old, T_new)
        
        self.x = float(T[0][2])  # X Global    metros
        self.y = float(T[1][2])  # Y Global
        self.z = float(navdata.altd)/1000.0       # Z Global

        ## Quaternion
        quaternion =  tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)

        ## Atualizacao do Yaw
        self.old_angle = self.yaw

        ## Publish
        self.publishPose (quaternion)
        self.publishPoseMarker(quaternion)


      # ********** Correcao Camera ************
      else:
        data = self.imageList.pop(0)

        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e 
        
        # Calibracao
        # Imagem Calibrada: 598x292 (X vs Y)
        cv_image = self.Calibrate(cv_image)

        # Initiate ORB Detection
        orb2 = cv2.ORB(3)

        # ORB Keypoints
        kp = orb2.detect(cv_image,None)

        # ORB Descriptors
        kp, des2 = orb2.compute(cv_image, kp)


  # ================= FUNCOES ===================

  ## Funcao de Calibracao da Imagem da Camera
  def Calibrate(self, cv_image):
    width = 640
    height = 360
    #calibration
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mat, self.dis, (width, height),1,(width, height))
    # undistort
    dst = cv2.undistort(cv_image, self.mat, self.dis, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    cv_image = dst[y:y+h, x:x+w]
    return cv_image

  ## Funcao de Mapeamento
  def Scale (self, old_min, old_max, new_min, new_max, old_value):
    if old_value > old_max:
        new_value = new_max

    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min)*new_range)/old_range) + new_min
    return new_value

  ## Funcao que desenha uma elipse da imagem
  def Ellipse (self, x, y, sx, sy, ro, c, img, color):
    alpha = 0.5*np.arctan2(2.0*ro*sx*sy,((sx*sx)-(sy*sy)))
    p1 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*cos(alpha)*sy*cos(alpha))-2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*sin(alpha)*sx*sin(alpha))))
    p2 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*sin(alpha)*sy*sin(alpha))+2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*cos(alpha)*sx*cos(alpha))))
    cv2.ellipse(img,(int(x),int(y)),(int(p1),int(p2)),degrees(alpha),0,360,color,1)

    return img

  ## Funcao que insere Posicao 3D da nova Feature no vetor de Features
  def insertFeature (self, pos3D):
    #self.featuresArray = np.row_stack ((self.featuresArray, pos3D))

    self.X = np.row_stack ((self.X, pos3D[0]))
    self.X = np.row_stack ((self.X, pos3D[1]))
    self.X = np.row_stack ((self.X, pos3D[2]))

    self.Qv = matrix.insertMatrix (self.Qv, np.zeros((3,3)))

    self.H = matrix.insertMatrixRight (self.H, 6)





  ## Funcao realiza Projecao 2D (u,v) a partir da Posicao 3D de uma Feature
  def Project_3D2D (self, pos_w):

    #Global to relative position transformation
    R_cw = transf.RotationMatrix3D (self.roll, self.pitch, self.yaw)
    P_cw = transf.Position3D (self.x, self.y, self.z)
    iH_cw = transf.InverseHomogeneousTransformation3D (R_cw, P_cw)

    R_pw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    P_pw = transf.Position3D (pos_w[0], pos_w[1], pos_w[2])
    H_pw = transf.HomogeneousTransformation3D (R_pw, P_pw)

    H_pc = np.dot(iH_cw, H_pw)

    relativePos = np.transpose(np.array([(H_pc[1][3]) , -(H_pc[2][3]) , (H_pc[0][3]) ]))   # metros

    # Position projected in the camera 
    #pos_2d = np.dot( self.mat_ret, relativePos )
    u = float(self.mat_ret[0][2]) + (float(self.mat_ret[0][0])*float(relativePos[0]))/float(relativePos[2])
    v = float(self.mat_ret[1][2]) + (float(self.mat_ret[1][1])*float(relativePos[1]))/float(relativePos[2])

    h = np.array([u, v])

    return h

  ## Funcao realiza Projecao 3D Global a partir de uma Posicao 2D (u,v em pixels) de uma Feature e uma Posicao z
  def Project_2D3D (self, u, v, z):
    #inv_M = matrix.inverseMatrix2(self.mat_ret)
    #aux = np.array([[u],[v],[1.0]])
    #pos3D_rel = z*(np.dot(inv_M, aux))     # metros

    x_rel = z*((u-self.mat_ret[0][2])/self.mat_ret[0][0])
    y_rel = z*((v-self.mat_ret[1][2])/self.mat_ret[1][1])

    #Global position transformation
    R_cw = transf.RotationMatrix3D (self.roll, self.pitch, self.yaw)
    P_cw = transf.Position3D (self.x, self.y, self.z)     # metros
    H_cw = transf.HomogeneousTransformation3D (R_cw, P_cw)

    R_pc = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    P_pc = transf.Position3D (z, x_rel, -y_rel)   # metros
    H_pc = transf.HomogeneousTransformation3D (R_pc, P_pc)

    H_pw = np.dot(H_cw, H_pc)

    # Posicao Global Feature
    pos3D = np.transpose(np.array([(H_pw[0][3]) , (H_pw[1][3]) , (H_pw[2][3]) ]))      # metros

    return pos3D

  ## Funcao retorna Posicao 3D Global estimada usando Bootstrap da Feature dada a posicao u e v em pixels e Posicao Z relativa
   # Insere Matriz de Covariancia da Feature na Matriz de Covariancia Geral
  def Bootstrap (self, u, v, z, max_samples):
    u_normal = np.random.normal(u, 2, max_samples)
    v_normal = np.random.normal(v, 2, max_samples)

    passo = float(z)/float(max_samples)

    sum_pos3D = np.transpose(np.array([0.0 , 0.0 , 0.0 ]))

    for i in range(1, (max_samples+1)):
      _z = i*passo
      _pos3D = self.Project_2D3D(u_normal[i-1], v_normal[i-1], _z)

      sum_pos3D += _pos3D

    pos3D = sum_pos3D/float(max_samples)


    bootstrapSum = np.array([[0.0 , 0.0 , 0.0 ],[0.0 , 0.0 , 0.0 ],[0.0 , 0.0 , 0.0 ]])

    for i in range(1, (max_samples+1)):
      _z = i*passo
      _pos3D = self.Project_2D3D(u_normal[i-1], v_normal[i-1], _z)

      aux = _pos3D - pos3D
      aux = np.array([[ aux[0], aux[1], aux[2] ]])

      bootstrapSum += np.dot(np.transpose(aux), aux)
      
    self.P_bootstr = (bootstrapSum/(float(max_samples)-2.0))
    self.P = matrix.insertMatrix(self.P, self.P_bootstr)

    return pos3D

  ## Funcao retorna Matriz de Projecao de Incerteza
  def covarianceMatrix (self, xf, yf, zf, index):
    yaw = self.X[5][0]
    pitch = self.X[4][0]
    roll = self.X[3][0]

    xc = float(self.X[0][0])
    yc = float(self.X[1][0])
    zc = float(self.X[2][0])

    Cx = float(self.mat_ret[0][2])
    Cy = float(self.mat_ret[1][2])
    fx = float(self.mat_ret[0][0])
    fy = float(self.mat_ret[1][1])

    sinR = sin(roll)
    sinP = sin(pitch)
    sinY = sin(yaw)
    cosR = cos(roll)
    cosP = cos(pitch)
    cosY = cos(yaw)

    R = np.eye((2))*2
    self.h_diff = np.array([
                      [(fx*(sinR*sinY + cosR*cosY*sinP)*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 + (fx*cosP*cosY)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR), 
                      (fx*cosP*sinY)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fx*(cosY*sinR - cosR*sinP*sinY)*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      (fx*cosP*cosR*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fx*sinP)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR),
                      (fx*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR)*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      - (fx*(zc*cosP - zf*cosP + xc*cosY*sinP - xf*cosY*sinP + yc*sinP*sinY - yf*sinP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fx*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY)*(zc*cosR*sinP - zf*cosR*sinP - xc*cosP*cosR*cosY + xf*cosP*cosR*cosY - yc*cosP*cosR*sinY + yf*cosP*cosR*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      (fx*(yc*cosP*cosY - yf*cosP*cosY - xc*cosP*sinY + xf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) + (fx*(xc*(cosY*sinR - cosR*sinP*sinY) - xf*(cosY*sinR - cosR*sinP*sinY) + yc*(sinR*sinY + cosR*cosY*sinP) - yf*(sinR*sinY + cosR*cosY*sinP))*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2
                      ],
                      [(fy*(sinR*sinY + cosR*cosY*sinP)*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fy*(cosR*sinY - cosY*sinP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR),
                      (fy*(cosR*cosY + sinP*sinR*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fy*(cosY*sinR - cosR*sinP*sinY)*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      (fy*cosP*sinR)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) + (fy*cosP*cosR*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      fy + (fy*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR)**2)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      - (fy*(zc*sinP*sinR - zf*sinP*sinR - xc*cosP*cosY*sinR + xf*cosP*cosY*sinR - yc*cosP*sinR*sinY + yf*cosP*sinR*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fy*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR)*(zc*cosR*sinP - zf*cosR*sinP - xc*cosP*cosR*cosY + xf*cosP*cosR*cosY - yc*cosP*cosR*sinY + yf*cosP*cosR*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2,
                      (fy*(xc*(cosY*sinR - cosR*sinP*sinY) - xf*(cosY*sinR - cosR*sinP*sinY) + yc*(sinR*sinY + cosR*cosY*sinP) - yf*(sinR*sinY + cosR*cosY*sinP))*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fy*(xc*(cosR*cosY + sinP*sinR*sinY) - xf*(cosR*cosY + sinP*sinR*sinY) + yc*(cosR*sinY - cosY*sinP*sinR) - yf*(cosR*sinY - cosY*sinP*sinR)))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)
                      ]
                      ])
    for x in xrange(0,(len(self.X)-6)/3):
      self.h_diff = matrix.insertMatrixRight(self.h_diff, 6)

    self.h_diff[0][6+((index-1)*3)] = - (fx*(sinR*sinY + cosR*cosY*sinP)*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fx*cosP*cosY)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)
    self.h_diff[0][7+((index-1)*3)] = (fx*(cosY*sinR - cosR*sinP*sinY)*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fx*cosP*sinY)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)
    self.h_diff[0][8+((index-1)*3)] = (fx*sinP)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fx*cosP*cosR*(zc*sinP - zf*sinP - xc*cosP*cosY + xf*cosP*cosY - yc*cosP*sinY + yf*cosP*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2
    self.h_diff[1][6+((index-1)*3)] = (fy*(cosR*sinY - cosY*sinP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fy*(sinR*sinY + cosR*cosY*sinP)*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2
    self.h_diff[1][7+((index-1)*3)] = (fy*(cosY*sinR - cosR*sinP*sinY)*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2 - (fy*(cosR*cosY + sinP*sinR*sinY))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)
    self.h_diff[1][8+((index-1)*3)] = - (fy*cosP*sinR)/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR) - (fy*cosP*cosR*(xc*(cosR*sinY - cosY*sinP*sinR) - xf*(cosR*sinY - cosY*sinP*sinR) - yc*(cosR*cosY + sinP*sinR*sinY) + yf*(cosR*cosY + sinP*sinR*sinY) - zc*cosP*sinR + zf*cosP*sinR))/(xc*(sinR*sinY + cosR*cosY*sinP) - xf*(sinR*sinY + cosR*cosY*sinP) - yc*(cosY*sinR - cosR*sinP*sinY) + yf*(cosY*sinR - cosR*sinP*sinY) + zc*cosP*cosR - zf*cosP*cosR)**2


    self.h_diff_T = np.transpose (self.h_diff)
    P_ij = np.dot( self.h_diff, self.P )
    P_ij = np.dot( P_ij, self.h_diff_T ) + R

    return P_ij

  ## Funcao retorna Distancia de Mahalanobis a partir da Matriz de Projecao de Incerteza e a Posicao 2D da nova Feature e a Feature do Mapa
  def mahalanobisDistance (self, P_ij, u_map, v_map, u, v):
    u_ij = u - u_map
    v_ij = v - v_map

    Mi = np.array ([[u_ij],[v_ij]])
    Mi_T = np.transpose(Mi)

    P_inv = matrix.inverseMatrix2 (P_ij)
    #P_inv = np.linalg.inv (P_ij)

    mDistance = np.dot(Mi_T, P_inv)
    mDistance = np.dot(mDistance, Mi)

    return mDistance


## ========= PUBLISHERS FUNCTIONS ==========

  ## Funcao que publica Posicao e Orientacao como mensagem Pose
  def publishPose (self, quaternion):
    self.pose.position.x = self.x*1000.0   #milimetros
    self.pose.position.y = self.y*1000.0
    self.pose.position.z = self.z*1000.0
    self.pose.orientation.x = quaternion[0]
    self.pose.orientation.y = quaternion[1]
    self.pose.orientation.z = quaternion[2]
    self.pose.orientation.w = quaternion[3]

    self.pub_pose_plot.publish(self.pose)

  ## Funcao que publica Posicao e Orientacao do robo como Marker
  def publishPoseMarker (self, quaternion):
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

    marker.pose.position.x = float(self.x)*1000.0      #milimetros
    marker.pose.position.y = float(self.y)*1000.0
    marker.pose.position.z = self.z*1000.0

    marker.id = self.counter
    self.counter += 1


    self.markerArray.markers.append(marker)

    self.pub_pose.publish(self.markerArray)


  ## Funcao que publica Posicao 3D das Features
  def publishFeaturesMarkers (self, pos3D):

    marker_feature = Marker()
    marker_feature.header.frame_id = "/neck"
    marker_feature.type = marker_feature.CUBE

    marker_feature.action = marker_feature.ADD
    marker_feature.scale.x = 30
    marker_feature.scale.y = 30
    marker_feature.scale.z = 30
    marker_feature.color.a = 1.0
    marker_feature.color.r = 1.0
    marker_feature.color.g = 0.0
    marker_feature.color.b = 1.0
    marker_feature.pose.orientation.w = 1.0


    marker_feature.pose.position.x = float(pos3D[0])*1000.0              #milimetros
    marker_feature.pose.position.y = float(pos3D[1])*1000.0
    marker_feature.pose.position.z = float(pos3D[2])*1000.0
    # marker_feature.pose.position.z = -self.Scale(292.0, 0.0, 0.0, 1300.0, float(pos3D[2])*1000.0)

    marker_feature.id = self.counter2


    self.markerArray_features.markers.append(marker_feature)

    # Publish the MarkerArray
    self.pub_features.publish(self.markerArray_features)


    self.counter2 += 1

  ## Funcao que modifica Posicao 3D das Features
  def correctFeaturesMarkers (self, pos3D, index):

    self.markerArray_features.markers[index].color.b = 0.0

    self.markerArray_features.markers[index].pose.position.x = float(pos3D[0])*1000.0              #milimetros
    self.markerArray_features.markers[index].pose.position.y = float(pos3D[1])*1000.0
    self.markerArray_features.markers[index].pose.position.z = float(pos3D[2])*1000.0

    # Publish the MarkerArray
    self.pub_features.publish(self.markerArray_features)



# ============== MAIN ================

def main(args):
  
  rospy.init_node('SLAM', anonymous=True)
  ic = SLAM()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)