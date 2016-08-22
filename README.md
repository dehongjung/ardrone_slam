# Monocular Visual SLAM based on EKF for Flying Robots

This repository contains the code used in the undergraduate thesis in Mechatronics Engineering, at the [University of Brasilia](http://www.unb.br), entitled "Monocular Visual SLAM based on EKF for Flying Robots".

The main goal of this project is developing a simultaneous localization and mapping system using IMU and camera informations from low-cost quadrotors. The system was based on the Extended Kalman Filter to manage and dynamically update its position and positions of features from the enviroment. The ORB algorithm was applied on this work to detect the features extracted from the monocular camera.

For this work, the flying robot used was the AR.Drone Parrot 2.0. It contains a frontal monocular camera and a embedded IMU. Also, the Parrot can transmit its camera and IMU data via Wi-Fi.

The code was written in Python using the ROS enviroment to manage the nodes and the quadrotor drivers.

Paper: https://drive.google.com/open?id=0B5QFVyyQ_lQ5Tk93UGZrR0lzUW8

Full Dissertation: https://drive.google.com/open?id=0B5QFVyyQ_lQ5bTR1Qmc4ZEtVeWM
