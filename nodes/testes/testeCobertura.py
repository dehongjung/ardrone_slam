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
import time
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

FeatureNum = 3
MAHALANOBIS_DISTANCE = 5.99

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
    self.mat_ret = np.array([[449.616018, 0.0, 282.673315], 
                             [0.0, 450.589432, 131.813266], 
                             [0.0, 0.0, 1.000000]])

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
    self.pub_pose_prediction = rospy.Publisher('pose3D_P', MarkerArray, queue_size=10)

    # Markers
    self.markerArray = MarkerArray()
    self.markerArray2 = MarkerArray()

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

    self.x2 = 0.0
    self.y2 = 0.0
    self.z2 = 0.0
    self.yaw2 = 0.0
    self.roll2 = 0.0
    self.pitch2 = 0.0
    self.old_angle2 = 0.0

    self.old_angle = 0.0
    self.old_x = 0.0
    self.old_y = 0.0
    self.old_vx = 0.0
    self.old_vy = 0.0
    self.initial_angle = 0.0


    # Matrizes de Projecao e Covariancia
    self.P_imu = np.array([
                          [0.0007, 0, 0, 0, 0, 0],
                          [0, 0.0007, 0, 0, 0, 0],
                          [0, 0, 0.0007, 0, 0, 0],
                          [0, 0, 0, 0.00001, 0, 0],
                          [0, 0, 0, 0, 0.00001, 0],
                          [0, 0, 0, 0, 0, 0.00001]
                          ])
    # self.P_imu = np.zeros((6,6))
    self.P = self.P_imu
    self.P_bootstr = np.zeros((3,3))
    # self.Qv = np.array([
    #                   [0.00004, 0, 0, 0, 0, 0],
    #                   [0, 0.00004, 0, 0, 0, 0],
    #                   [0, 0, 0.00004, 0, 0, 0],
    #                   [0, 0, 0, 0.0003, 0, 0],
    #                   [0, 0, 0, 0, 0.0003, 0],
    #                   [0, 0, 0, 0, 0, 0.0003]
    #                   ])
    self.Qv = np.array([
                      [0.0009, 0, 0, 0, 0, 0],
                      [0, 0.0009, 0, 0, 0, 0],
                      [0, 0, 0.0009, 0, 0, 0],
                      [0, 0, 0, 0.0003, 0, 0],
                      [0, 0, 0, 0, 0.0003, 0],
                      [0, 0, 0, 0, 0, 0.0003]
                      ])

    self.R = np.array([
                      [0.000001, 0, 0],
                      [0, 0.000001, 0],
                      [0, 0, 0.000001],
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
    self.counter3 = 0
    
    # Variaveis de Sincronizacao
    self.semaphore = 0
    self.sync_counter = 0

    # Listas
    self.controlList = []
    self.navdataList = []
    self.imageList = []
  
    # Vetores Tinder
    self.noMatchCounterMap = []
    self.matchCounter = []

    self.init_time = rospy.get_time()

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
      print rospy.get_time() - self.init_time

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

        ## Apenas Predicao
        self.predictionStep (navdata, delta_t)
       
        #self.dt = 0.023

        ## Atualizacao da Orientacao
        self.yaw = self.X[5][0]
        self.pitch = self.X[4][0]
        self.roll = self.X[3][0]

        ## Calculo de deslocamento relativo X e Y
        x = float((navdata.vx)*delta_t + 0.5*navdata.ax*9.806*(delta_t*delta_t))/1000.0   #metros
        y = float((navdata.vy)*delta_t + 0.5*navdata.ay*9.806*(delta_t*delta_t))/1000.0
        
        ## Tranformacao de coordenadas X e Y (Relativa -> Global)
        R_old = transf.RotationMatrix2D(self.old_angle)
        p_old = transf.Position2D(self.X[0][0], self.X[1][0])
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

        ## Predicao do Vetor de Estados
        self.X[0][0] = self.x
        self.X[1][0] = self.y
        self.X[2][0] = self.z
        self.X[3][0] = self.roll
        self.X[4][0] = self.pitch
        self.X[5][0] = self.yaw

        # print "1: ", self.X[5][0]

        ## Predicao da Matriz de Covariancia
        self.P = self.P + self.Qv

        ## ********** Correcao Navdata *************
        Ht = np.transpose(self.H)
        G = np.dot(self.P, Ht)
        aux = np.dot(self.H, self.P)
        aux = np.linalg.inv(np.dot(aux, Ht) + self.R)

        # Ganho de Kalman
        G = np.dot(G, aux)

        # Correcao do Vetor de Estados
        aux = np.array([[radians(navdata.rotX)], [radians(navdata.rotY)], [radians(navdata.rotZ)]]) - np.array([[self.X[3][0]], [self.X[4][0]], [self.X[5][0]]])
        self.X = self.X + np.dot(G, aux)

        # print "2: ", G

        # Correcao da Matriz de Covariancia
        aux = np.eye((len(self.P))) - np.dot(G, self.H)
        self.P = np.dot(aux, self.P)

        # print self.H

        ## Plotar os graficos
        self.x = self.X[0][0]
        self.y = self.X[1][0]
        self.z = self.X[2][0]
        self.roll = self.X[3][0]
        self.pitch = self.X[4][0]
        self.yaw = self.X[5][0]

        ## Publish
        quaternion =  tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
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
        orb2 = cv2.ORB(FeatureNum)

        # ORB Keypoints
        self.kp = orb2.detect(cv_image,None)

        # ORB Descriptors
        self.kp, des2 = orb2.compute(cv_image, self.kp)

        # Draw Keypoints
        self.img = cv2.drawKeypoints(cv_image,self.kp,color=(0,255,0), flags=0)

        #contador de NAO MATCHES auxiliar (Se for igual a 3, soma 1 no outro)
        self.noMatchCounterMapAux = []
        for j in range(0, len(self.X)):
          self.noMatchCounterMapAux.append(0)

        ## Correcao da Camera

        if (len(self.kp) > 0):
          for x in xrange(0, len(self.kp)):
            
            # Feature Observada (u,v)
            u = self.kp[x].pt[0]
            v = self.kp[x].pt[1]

            # Feature Inicial
            if (self.init):
              pos3D_newF = self.Bootstrap (u, v, 2.0, 100)
              self.insertFeature (pos3D_newF)
              # Publica nova Posicao 3D da feature do mapa como Marker 
              self.publishFeaturesMarkers(pos3D_newF)
              self.init = 0

              self.noMatchCounterMapAux.append(0)
              self.noMatchCounterMapAux.append(0)
              self.noMatchCounterMapAux.append(0)

              self.matchCounter.append(0)

            # Features Posteriores
            else:
              # Matching
              match = 0
              minDistance = 99999.0
              for k in range(6, len(self.X), 3):
                # Define Posicao 3D da Feature k
                pos3D_f = transf.Position3D(float(self.X[k][0]), float(self.X[k+1][0]), float(self.X[k+2][0]))

                modulo = sqrt(((self.X[0][0] - self.X[k][0])**2) + ((self.X[1][0] - self.X[k+1][0])**2))
                if modulo >= 5.0:
                  continue
                # Projeta a Posicao 3D da Feature k do Mapa para 2D (u,v)
                h = self.Project_3D2D (pos3D_f) 
                # self.img = self.Ellipse(h[0], h[1], 5.0, 5.0, 0.87, 9, self.img, (255, 0, 0))

                # Verifica se a Projecao 2D esta fora da janela
                if (h[1]>291 or h[1]<0 or h[0]>597 or h[0]<0):
                  # self.changeColorFeatureBlue(k)
                  continue

                # Caso a Projecao 2D esteja dentro da janela  
                else: 
                  # Retorna Matrix de Projecao de Incerteza da Feature k no Mapa
                  P_ij = self.covarianceMatrix (float(self.X[k][0]), float(self.X[k+1][0]), float(self.X[k+2][0]), ((k-6)/3)+1)
                  # self.changeColorFeatureGreen(k)

                  # Calcula a Distancia de Mahalanobis entre a Feature medida e a Feture k do Mapa
                  mDistance = self.mahalanobisDistance (P_ij, float(h[0]), float(h[1]), u, v)

                  # print mDistance
                  #bDistance = self.bruteForceDistance (float(h[0]), float(h[1]), u, v)
                  #print bDistance
                  # Not Matched
                  if (mDistance > MAHALANOBIS_DISTANCE):
                    # aumenta contador do mapa auxiliar
                    self.noMatchCounterMapAux[k] = self.noMatchCounterMapAux[k] + 1
                    # self.img = self.Ellipse(h[0], h[1], 5.0, 5.0, 0.87, 9, self.img, (200, 0, 0))
                    continue

                  # Matched
                  else:
                    # self.img = self.Ellipse(h[0], h[1], 5.0, 5.0, 0.87, 9, self.img, (0, 0, 255))
                    match = 1
                    self.matchCounter[(k-6)/3] = self.matchCounter[(k-6)/3] + 10
                    contador_temporario = k + 3
                    while (contador_temporario < len(self.X)):
                      # self.changeColorFeatureBlue(contador_temporario)
                      contador_temporario = contador_temporario + 3

                    break

                    # if mDistance < minDistance:
                    #   minDistance = mDistance
                    #   self.noMatchCounterMapAux[match] = self.noMatchCounterMapAux[match] + 1
                    #   match = k
                      # print "X: ", x
                      # print k
                      # print minDistance

              # Adicao no Mapa
              if(match == 0):
                # Calcula a Matrix de Covariancia da nova Feature, insere na Matriz de Covariancia Geral e retorna a sua Posicao 3D
                pos3D_newF = self.Bootstrap (u, v, 2.0, 100)

                # Insere a nova feature no vetor de features do mapa
                self.insertFeature (pos3D_newF)

                # Insere 0 no contador auxiliar 
                self.noMatchCounterMapAux.append(0)
                self.noMatchCounterMapAux.append(0)
                self.noMatchCounterMapAux.append(0)

                # Publica nova Posicao 3D da feature do mapa como Marker 
                self.publishFeaturesMarkers(pos3D_newF)

                self.matchCounter.append(0)

                # print "Nao deu match!"


              # Correcao
              else:
                # k = match

                #Zerar contador 
                self.noMatchCounterMap[(k-6)/3] = 0

                # Ganho de Kalman K
                _K = np.dot(self.P, self.h_diff_T)
                K = np.dot(_K, matrix.inverseMatrix2(P_ij))

                # Correcao do Vetor de Estados
                aux = np.array([[u],[v]]) - np.array([[h[0]], [h[1]]])
                self.X = self.X + np.dot(K, aux)

                # Correcao da Matriz de Covariancia
                self.P = np.dot(np.eye((len(self.P))) - np.dot(K, self.h_diff), self.P)
                index = (k-6)/3
                # Correcao no Publish
                self.correctFeaturesMarkers(k)

              for i in range (0, len(self.matchCounter)):
                self.matchCounter[i] = self.matchCounter[i] + 1

      # Eliminacao de Features
      # for k in range(6, len(self.X), 3):
      k = 6
      while (k < len(self.X)):
        if k >= len(self.X):
          break
        # Verifica o contador do mapa
        if self.noMatchCounterMapAux[k] >= len(self.kp):
          # print "K: ", k
          self.noMatchCounterMap[(k-6)/3] = self.noMatchCounterMap[(k-6)/3] + 1

          if self.noMatchCounterMap[(k-6)/3] > 60:
            # Se maior que 20
            self.deleteFeature(k)
            self.noMatchCounterMapAux.pop(k)
            self.noMatchCounterMapAux.pop(k)
            self.noMatchCounterMapAux.pop(k)
            self.deletePublishFeaturesMarkers((k-6)/3)
            continue
            k = k - 3
        

        if self.matchCounter[(k-6)/3] == 3:
          # Se maior que 20
          self.deleteFeature(k)
          self.noMatchCounterMapAux.pop(k)
          self.noMatchCounterMapAux.pop(k)
          self.noMatchCounterMapAux.pop(k)
          self.deletePublishFeaturesMarkers((k-6)/3)
          k = k - 3

        k = k + 3


        # print "-----------"
        # print "X", self.X
        # Show Image
        # cv2.imshow("ORB", self.img)
        # time.sleep(0.2)
        # cv2.waitKey(1)
      # if len(self.X) > 400:
      #self.publishAllFeaturesMarkers()


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

    self.noMatchCounterMap.append(0)

  def deleteFeature (self, k):
    # Tira do Vetor de Estados
    self.X = np.delete(self.X, k, axis=0)
    self.X = np.delete(self.X, k, axis=0)
    self.X = np.delete(self.X, k, axis=0)
    # Tira da Matriz de Covariancia
    self.P = matrix.removeMatrix(self.P,1+((k-6)/3))
    # Tira do Qv
    self.Qv = matrix.removeMatrix(self.Qv,1+((k-6)/3))
    # Tira do H
    self.H = matrix.removeMatrixRight(self.H, k)
    # Tira do noMatchCounter 
    self.noMatchCounterMap.pop((k-6)/3)

    self.matchCounter.pop((k-6)/3)

  def predictionStep (self, navdata, delta_t):
    ## Atualizacao da Orientacao
    self.yaw2 = radians(navdata.rotZ)
    self.pitch2 = radians(navdata.rotY)
    self.roll2 = radians(navdata.rotX)

    ## Calculo de deslocamento relativo X e Y
    x = float((navdata.vx)*delta_t + 0.5*navdata.ax*9.806*(delta_t*delta_t))/1000.0   #metros
    y = float((navdata.vy)*delta_t + 0.5*navdata.ay*9.806*(delta_t*delta_t))/1000.0
    
    ## Tranformacao de coordenadas X e Y (Relativa -> Global)
    R_old = transf.RotationMatrix2D(self.old_angle2)
    p_old = transf.Position2D(self.x2, self.y2)
    T_old = transf.HomogeneousTransformation2D(R_old, p_old)

    R_new = transf.RotationMatrix2D(self.yaw2)
    p_new = transf.Position2D(x, y)
    T_new = transf.HomogeneousTransformation2D(R_new, p_new)

    T = np.dot(T_old, T_new)
    
    self.x2 = float(T[0][2])  # X Global    metros
    self.y2 = float(T[1][2])  # Y Global
    self.z2 = float(navdata.altd)/1000.0       # Z Global

    ## Quaternion
    quaternion =  tf.transformations.quaternion_from_euler(self.roll2, self.pitch2, self.yaw2)

    ## Atualizacao do Yaw
    self.old_angle2 = self.yaw2

    ## Publish
    self.publishPoseMarkerPrediction(quaternion)

  ## Funcao realiza Projecao 2D (u,v) a partir da Posicao 3D de uma Feature
  def Project_3D2D (self, pos_w):

    # Transformacao de coordenadas do robo para a camera
    R_rw = transf.RotationMatrix3D (self.X[3][0], self.X[4][0], self.X[5][0])
    P_rw = transf.Position3D (self.X[0][0], self.X[1][0], self.X[2][0])
    H_rw = transf.HomogeneousTransformation3D (R_rw, P_rw)

    p_cr = np.array([[0.2],[0.0],[0.0],[1.0]])
    p_cw = np.dot(H_rw, p_cr)

    #Global to relative position transformation
    R_cw = transf.RotationMatrix3D (self.X[3][0], self.X[4][0], self.X[5][0])
    P_cw = transf.Position3D (p_cw[0][0], p_cw[1][0], p_cw[2][0])
    iH_cw = transf.InverseHomogeneousTransformation3D (R_cw, P_cw)

    R_pw = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    P_pw = transf.Position3D (pos_w[0], pos_w[1], pos_w[2])
    H_pw = transf.HomogeneousTransformation3D (R_pw, P_pw)

    H_pc = np.dot(iH_cw, H_pw)

    relativePos = np.transpose(np.array([(-H_pc[1][3]) , -(H_pc[2][3]) , (H_pc[0][3]) ]))   # metros

    # Position projected in the camera 
    #pos_2d = np.dot( self.mat_ret, relativePos )
    u = float(self.mat_ret[0][2]) + (float(self.mat_ret[0][0])*float(relativePos[0]))/float(relativePos[2])
    v = float(self.mat_ret[1][2]) + (float(self.mat_ret[1][1])*float(relativePos[1]))/float(relativePos[2])

    if relativePos[2] < 0:
      u = -999.0

    h = np.array([u, v])

    return h

  ## Funcao realiza Projecao 3D Global a partir de uma Posicao 2D (u,v em pixels) de uma Feature e uma Posicao z
  def Project_2D3D (self, u, v, z):
    #inv_M = matrix.inverseMatrix2(self.mat_ret)
    #aux = np.array([[u],[v],[1.0]])
    #pos3D_rel = z*(np.dot(inv_M, aux))     # metros

    x_rel = z*((u-self.mat_ret[0][2])/self.mat_ret[0][0])
    y_rel = z*((v-self.mat_ret[1][2])/self.mat_ret[1][1])

    # Transformacao de coordenadas do robo para a camera
    R_rw = transf.RotationMatrix3D (self.X[3][0], self.X[4][0], self.X[5][0])
    P_rw = transf.Position3D (self.X[0][0], self.X[1][0], self.X[2][0])
    H_rw = transf.HomogeneousTransformation3D (R_rw, P_rw)

    p_cr = np.array([[0.2],[0.0],[0.0],[1.0]])
    p_cw = np.dot(H_rw, p_cr)

    #Global position transformation
    R_cw = transf.RotationMatrix3D (self.X[3][0], self.X[4][0], self.X[5][0])
    P_cw = transf.Position3D (p_cw[0][0], p_cw[1][0], p_cw[2][0])     # metros
    H_cw = transf.HomogeneousTransformation3D (R_cw, P_cw)

    R_pc = transf.RotationMatrix3D (0.0, 0.0, 0.0)
    # P_pc = transf.Position3D (z, -y_rel, x_rel)   # metros
    P_pc = transf.Position3D (z, -x_rel, -y_rel)   # metros
    H_pc = transf.HomogeneousTransformation3D (R_pc, P_pc)

    H_pw = np.dot(H_cw, H_pc)

    # Posicao Global Feature
    pos3D = np.transpose(np.array([(H_pw[0][3]) , (H_pw[1][3]) , (H_pw[2][3]) ]))      # metros    # MODIFICADO

    return pos3D

  ## Funcao retorna Posicao 3D Global estimada usando Bootstrap da Feature dada a posicao u e v em pixels e Posicao Z relativa
   # Insere Matriz de Covariancia da Feature na Matriz de Covariancia Geral
  def Bootstrap (self, u, v, z, max_samples):
    u_normal = np.random.normal(u, 1, max_samples)
    v_normal = np.random.normal(v, 1, max_samples)

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
    self.P           = matrix.insertMatrix(self.P, self.P_bootstr)

    return pos3D

  ## Funcao retorna Matriz de Projecao de Incerteza
  def covarianceMatrix (self, xf, yf, zf, index):
    yaw = self.X[5][0]
    pitch = self.X[4][0]
    roll = self.X[3][0]

    # Transformacao de coordenadas do robo para a camera
    R_rw = transf.RotationMatrix3D (self.X[3][0], self.X[4][0], self.X[5][0])
    P_rw = transf.Position3D (self.X[0][0], self.X[1][0], self.X[2][0])
    H_rw = transf.HomogeneousTransformation3D (R_rw, P_rw)

    p_cr = np.array([[0.2],[0.0],[0.0],[1.0]])
    p_cw = np.dot(H_rw, p_cr)

    xc = float(p_cw[0][0])
    yc = float(p_cw[1][0])
    zc = float(p_cw[2][0])

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

    # White Noise
    R = np.eye((2))*5

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

    if abs(u_ij) > 50 or abs(v_ij) > 50:
      return 999.99

    Mi = np.array ([[u_ij],[v_ij]])
    Mi_T = np.transpose(Mi)

    P_inv = matrix.inverseMatrix2 (P_ij)
    #P_inv = np.linalg.inv (P_ij)

    mDistance = np.dot(Mi_T, P_inv)
    mDistance = np.dot(mDistance, Mi)

    return mDistance

  def bruteForceDistance (self, u_map, v_map, u, v):
    u_ij = u - u_map
    v_ij = v - v_map

    distance = sqrt((u_ij**2) + (v_ij**2))

    return distance


## ========= PUBLISHERS FUNCTIONS ==========

  ## Funcao que publica Posicao e Orientacao como mensagem Pose
  def publishPose (self, quaternion):
    self.pose.position.x = self.X[0][0]*1000.0   #milimetros
    self.pose.position.y = self.X[1][0]*1000.0
    self.pose.position.z = self.X[2][0]*1000.0
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

    marker.pose.position.x = float(self.X[0][0])*1000.0      #milimetros
    marker.pose.position.y = float(self.X[1][0])*1000.0
    marker.pose.position.z = self.X[2][0]*1000.0

    marker.id = self.counter

    self.counter += 1

    self.markerArray.markers.append(marker)

    self.pub_pose.publish(self.markerArray)

  def publishPoseMarkerPrediction (self, quaternion):
    marker = Marker()
    marker.header.frame_id = "/neck"
    marker.type = marker.ARROW

    marker.action = marker.ADD
    marker.scale.x = 40
    marker.scale.y = 10
    marker.scale.z = 6
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.pose.position.x = float(self.x2)*1000.0      #milimetros
    marker.pose.position.y = float(self.y2)*1000.0
    marker.pose.position.z = float(self.z2)*1000.0

    marker.id = self.counter3

    self.counter3 += 1

    self.markerArray2.markers.append(marker)

    self.pub_pose_prediction.publish(self.markerArray2)



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

    marker_feature.id = self.counter2


    self.markerArray_features.markers.append(marker_feature)

    # Publish the MarkerArray
    self.pub_features.publish(self.markerArray_features)


    self.counter2 += 1


  def deletePublishFeaturesMarkers (self, index):
    # self.markerArray_features.markers.pop(index)

    self.markerArray_features.markers[index].action = 2

    self.pub_features.publish(self.markerArray_features)

    self.markerArray_features.markers.pop(index)



  ## Funcao que modifica Posicao 3D das Features
  def correctFeaturesMarkers (self, index):

    for i in range(6,len(self.X), 3):
      if i == index:
        self.markerArray_features.markers[(i-6)/3].color.g = self.markerArray_features.markers[(index-6)/3].color.g + 0.08
        self.markerArray_features.markers[(i-6)/3].color.b = 1
        self.markerArray_features.markers[(i-6)/3].color.r = 1

      self.markerArray_features.markers[(i-6)/3].pose.position.x = float(self.X[i][0])*1000.0              #milimetros
      self.markerArray_features.markers[(i-6)/3].pose.position.y = float(self.X[i+1][0])*1000.0
      self.markerArray_features.markers[(i-6)/3].pose.position.z = float(self.X[i+2][0])*1000.0

    # Publish the MarkerArray
    self.pub_features.publish(self.markerArray_features)

  def changeColorFeatureGreen (self, index): 
    self.markerArray_features.markers[(index-6)/3].color.g = 1
    self.markerArray_features.markers[(index-6)/3].color.b = 0
    self.markerArray_features.markers[(index-6)/3].color.r = 0
  def changeColorFeatureBlue (self, index): 
    self.markerArray_features.markers[(index-6)/3].color.g = 0
    self.markerArray_features.markers[(index-6)/3].color.b = 1
    self.markerArray_features.markers[(index-6)/3].color.r = 0


  def publishAllFeaturesMarkers (self):

    for i in range(6,len(self.X), 3):

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

      marker_feature.pose.position.x = float(self.X[i][0])*1000.0              #milimetros
      marker_feature.pose.position.y = float(self.X[i+1][0])*1000.0 
      marker_feature.pose.position.z = float(self.X[i+2][0])*1000.0 

      marker_feature.id = (i-6)/3

      self.markerArray_features.markers.append(marker_feature)

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