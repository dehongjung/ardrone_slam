#!/usr/bin/env python

'''
	Universidade de Brasilia
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Localizacao 3D e Deteccao de Features ORB do Quadrirotor AR.Drone Parrot 2.0
'''

# =============== Bibliotecas ===================

import roslib
roslib.load_manifest('ardrone_tutorials')

import sys
import rospy
import cv2
import tf
import numpy as np
import message_filters

from math import sin, cos, radians, sqrt, degrees

from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from find_obj import filter_matches,explore_match
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

    # Janelas
    cv2.namedWindow("ORB", 1)

    # Subscriber
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.DetectFeatures)
    # image_sub = message_filters.Subscriber("/ardrone/front/image_raw", Image)

    # Publisher
    self.pub_image = rospy.Publisher("/camera/image_raw", Image)
    self.pub_features = rospy.Publisher('features', MarkerArray)
    self.markerArray_features = MarkerArray()

    # CvBridge
    self.bridge = CvBridge()

    # Calibracao
    self.mat = np.array([[582.839034, 0.000000, 305.560522], [0.000000, 582.302062, 173.607018], [0.000000, 0.000000, 1.000000]])
    self.dis = np.array([-0.517441, 0.270697, -0.003375, 0.001576, 0.000000])

    # Variaveis
    self.count = 1 
    self.counter2 = 0

    #########################
    ##     Localization    ##
    #########################

    self.yaw = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    # Janelas
    cv2.namedWindow("Pose", 1)
    cv2.namedWindow("O_P", 1)

    # Subscriber
    self.sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.Localization)
    # sub_Navdata = message_filters.Subscriber('/ardrone/navdata', Navdata)

    # Publisher
    self.pub_pose = rospy.Publisher('pose3D', MarkerArray)
    self.markerArray = MarkerArray()

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
    self.counter = 0
    self.MARKERS_MAX = 5000
    self.z = 0.0
    self.theta_new = 0.0

    # Escala
    for counter in range(0, 1000, 50):
        cv2.line(self.img, (counter,0), (counter,1000), (0,0,0), 1, 4, 0)
        cv2.line(self.img, (1000, counter), (0,counter), (0,0,0), 1, 4, 0)


    cv2.circle(self.img, (500,500), 6, (255,0,0), 3, 8, 0)



    # self.ts = message_filters.TimeSynchronizer([image_sub, sub_Navdata], 10)
    # self.ts.registerCallback(self.callback)


  # ================= LOOP ===================

  def callback(self, image, navdata):
    print "hello"

  def Localization(self, navdata):

    self.yaw = (navdata.rotZ)
    self.pitch = (navdata.rotY)
    self.roll = (navdata.rotX)

    ## Tempo de Amostragem
    if (self.aux2):
      self.t = navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)
      self.aux2 = 0
    
    self.dt = (navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)) - self.t
    self.t = navdata.header.stamp.secs + (navdata.header.stamp.nsecs/1000000000.0)
    delta_t = self.dt
   
    #self.dt = 0.023

    ## Inicializacao da Orientacao (Yaw)
    if(self.aux):
        self.initial_angle = navdata.rotZ
        self.aux = 0
    

    #delta_t = 0.02

    ## Definicao do novo theta
    self.theta_new = navdata.rotZ - self.initial_angle

    ## Calculo de deslocamento relativo X e Y
    x = (navdata.vx)*delta_t + 0.5*navdata.ax*9806*(delta_t*delta_t)
    y = (navdata.vy)*delta_t + 0.5*navdata.ay*9806*(delta_t*delta_t)
    
    ## Tranformacao de coordenadas X e Y (Relativa -> Global)
    T_old = np.array([
                      [cos(self.old_angle), -sin(self.old_angle), self.old_x], 
                      [sin(self.old_angle), cos(self.old_angle), self.old_y], 
                      [0,0,1] ])
    T_new = np.array([
                      [cos(radians(self.theta_new)), -sin(radians(self.theta_new)), x], 
                      [sin(radians(self.theta_new)), cos(radians(self.theta_new)), y], 
                      [0,0,1]])
    T = np.dot(T_old, T_new)
    
    self.position[0] = T[0][2]  # X Global
    self.position[1] = T[1][2]  # Y Global
    self.z = navdata.altd       # Z Global
    
    ## Escrita de Dados de Posicao e Tempo em um arquivo txt
    limit = 2500  # Limitador para parar a escrita
    #self.WriteFile(navdata.altd, limit)                            #TIREI POR ENQUANTO

    ## Print
    # print 'Posicao X: ', self.position[0]
    # print 'Posicao Y: ', self.position[1]
    # print 'Posicao Z: ', navdata.altd
    # print 'Angulo Z: ', self.theta_new
    # print 'Velocidade X: ', navdata.vx
    # print 'Velocidade Y: ', navdata.vy
    # print 'Angulo Inicial: ', self.initial_angle
    # print '---------------------------'


    ## Vizualizacao

    cor = self.Scale(100, 50000, 0, 255, navdata.altd)

    # Desenho da Posicao X e Y estimada (ponto vermelho = takeoff)
    if navdata.altd != 0:
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

    cv2.line(self.img2, (50,50), (50-int(20*sin(radians(self.theta_new))), 50-int(20*cos(radians(self.theta_new)))), (0,0,255), 2, 8, 0)
    cv2.circle(self.img2, (50,50), 1, (255,0,0), 2, 8, 0)  

    # Janelas
    cv2.imshow("Pose",self.img)
    cv2.imshow("O_P",self.img2)
    cv2.waitKey(1)
    
    ## Atualizacao das varaveis de posicao
    self.old_x = self.position[0]
    self.old_y = self.position[1]
    self.old_angle = radians(self.theta_new)

    ## Quaternion
    quaternion =  tf.transformations.quaternion_from_euler(radians(navdata.rotX), radians(navdata.rotY), radians(self.theta_new))

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
    marker.pose.position.z = navdata.altd

    marker.id = self.counter
    self.counter += 1

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
    self.pub_pose.publish(self.markerArray)


  def DetectFeatures(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e 
    
    # Calibracao
    # Imagem Calibrada: 598x292 (X vs Y)
    # cv_image = self.Calibrate(cv_image)

    # Publisher
    image_pub = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    self.pub_image.publish(image_pub)

    # Initiate STAR detector
    orb2 = cv2.ORB(10)
    # find the keypoints with ORB
    kp2 = orb2.detect(cv_image,None)

    # compute the descriptors with ORB
    kp2, des2 = orb2.compute(cv_image, kp2)

    # print "X: ", kp2[0].pt[0]
    # print "Y: ", kp2[0].pt[1]


    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(cv_image,kp2,color=(0,255,0), flags=0)

    ################
    ## Project 2D ##
    ################
    Pf = np.array([[0.0],
                  [2400.0],
                  [1.0]])
    X = np.array([
                  [cos(self.theta_new), sin(self.theta_new), -(self.position[0]*cos(self.theta_new) + self.position[1]*sin(self.theta_new))],
                  [-sin(self.theta_new), cos(self.theta_new), -(-self.position[0]*sin(self.theta_new) + self.position[1]*cos(self.theta_new))],
                  [0,0,1]])
    relativePos = np.dot(X, Pf)

    # relativePositionX = 0.0+(self.position[0]/1000.0)
    # relativePositionZ =  2.4-(self.position[1]/1000.0)
    # relativePositionY = -1.0+(self.z/1000.0)

    relativePositionX = relativePos[0][0]/1000.0
    relativePositionZ = relativePos[1][0]/1000.0
    relativePositionY = self.z/1000.0 - 1.0


    axis = np.float32([[int(relativePositionX), int(relativePositionY), int(relativePositionZ)]]).reshape(-1,3)
    img2 = self.Project_3D2D (img2, axis)

    cv2.imshow("ORB", img2)


    # # BFMatcher with default params
    # bf = cv2.BFMatcher()
    # matches = bf.knnMatch(self.des1,des2, k=2)

    # # Apply ratio test
    # good = []
    # count = 0
    # for m,n in matches:
    #   if m.distance < 0.8*n.distance:
    #     count = count + 1
    #     good.append([m])

    # print "Numero de Matches: ", count
    # print "Numero de Features Atual: ", len(des2)
    # print "Numero de Features Inicial: ", len(self.des1)
    # print "--------------------"


    ########################
    ## Publisher features ##
    ########################

    for num in range(0, len(kp2)):
      marker_feature = Marker()
      marker_feature.header.frame_id = "/neck"
      marker_feature.type = marker_feature.CUBE

      marker_feature.action = marker_feature.ADD
      marker_feature.scale.x = 10
      marker_feature.scale.y = 10
      marker_feature.scale.z = 10
      marker_feature.color.a = 1.0
      marker_feature.color.r = 1.0
      marker_feature.color.g = 0.0
      marker_feature.color.b = 1.0
      marker_feature.pose.orientation.w = 1.0

      ## Tranformacao de coordenadas X e Y (Relativa -> Global)
      T_old = np.array([
                        [cos(self.old_angle), -sin(self.old_angle), self.position[0]], 
                        [sin(self.old_angle), cos(self.old_angle), self.position[1]], 
                        [0,0,1] ])
      T_new = np.array([
                        [cos(radians(0.0)), -sin(radians(0.0)), 1600], 
                        [sin(radians(0.0)), cos(radians(0.0)), self.Scale(0.0, 598.0, 0.0, 2000.0, kp2[num].pt[0]) - 1000.0], 
                        [0,0,1]])
      T = np.dot(T_old, T_new)
      
      x_feature = T[0][2]  # X Global
      y_feature = T[1][2]  # Y Global


      marker_feature.pose.position.x = float(x_feature)
      marker_feature.pose.position.y = float(y_feature)
      marker_feature.pose.position.z = -self.Scale(292.0, 0.0, 0.0, 1300.0, self.z)

      marker_feature.id = self.counter2

      # self.kp_array.append(kp2[num])
      # self.kp_array[2]


      # if(self.counter > self.MARKERS_MAX):
      #     self.markerArray_features.markers.pop(1)

      self.markerArray_features.markers.append(marker_feature)

      # Publish the MarkerArray
      self.pub_features.publish(self.markerArray_features)

      self.counter2 += 1

    

    cv2.waitKey(3)


  # ================= FUNCOES ===================

  def Calibrate(self, cv_image):
    self.w = 640
    self.h = 360
    #calibration
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mat, self.dis, (self.w,self.h),1,(self.w,self.h))
    # undistort
    dst = cv2.undistort(cv_image, self.mat, self.dis, None, newcameramtx)
    # crop the image
    x,y,w,h = roi
    cv_image = dst[y:y+h, x:x+w]
    return cv_image

  def Scale (self, old_min, old_max, new_min, new_max, old_value):
    if old_value > old_max:
        new_value = new_max

    old_range = old_max - old_min
    new_range = new_max - new_min
    new_value = (((old_value - old_min)*new_range)/old_range) + new_min
    return new_value

  def Ellipse (self, x, y, sx, sy, ro, c, img, color):
    alpha = 0.5*np.arctan2(2.0*ro*sx*sy,((sx*sx)-(sy*sy)))
    p1 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*cos(alpha)*sy*cos(alpha))-2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*sin(alpha)*sx*sin(alpha))))
    p2 = sqrt( c*((sx*sx)*(sy*sy)*(1.0-(ro*ro)))/((sy*sin(alpha)*sy*sin(alpha))+2.0*ro*sx*sy*sin(alpha)*cos(alpha)+(sx*cos(alpha)*sx*cos(alpha))))
    cv2.ellipse(img,(int(x),int(y)),(int(p1),int(p2)),degrees(alpha),0,360,color,1)

    return img

  def Project_3D2D (self, img, axis):
    #rvecs  = np.float32([[self.roll,self.pitch,self.yaw]]) 
    rvecs  = np.float32([[0,0,0]])
    tvecs  = np.float32([[0,0,0]])
    # project 3D points to image plane
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.mat, self.dis)

    img = self.Ellipse(imgpts[0][0][0], imgpts[0][0][1], 5.0, 5.0, 0.87, 9, img, (0, 255, 0))

    return img


# ============== MAIN ================

def main(args):
  ic = SLAM()
  rospy.init_node('SLAM', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)