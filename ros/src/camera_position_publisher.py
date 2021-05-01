#!/usr/bin/env python3

import cv2
from cv2 import aruco
import numpy as np
from math import cos, sin, atan, atan2
from utils import *
# from autonomous_car_model.srv import car_pose
from autonomous_car_model.msg import CarPose
import rospy

class Aruco:
    def __init__(self):
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL )
        self.arucoParam = aruco.DetectorParameters_create()
        self.markerSize = 0.1
       
        self.cap = cv2.VideoCapture(0)

        self.cameraMatrix, self.distCoef = self.loadCalibrationMatrixCoef()


        #test
        self.cameraClient = CameraClient()
        self.cameraClient.connect()

        rospy.init_node("aruco_camera")
        self.pub = rospy.Publisher("aruco_position", CarPose, queue_size = 10)
        self.rate = rospy.Rate(2)
        self.carPose = CarPose()

    def loadCalibrationMatrixCoef(self):
        f = open("camera_calibration_matrix.csv")

        calibration_matrix = np.zeros((3* 3, ))
        dist_coef = np.zeros((5,))
        line = f.readline()

        for i, str_value in enumerate(line.split(sep=";")):
            if i<9:
                calibration_matrix[i] = float(str_value)
            else:
                dist_coef[i-9] = float(str_value)
        calibration_matrix = calibration_matrix.reshape(3, 3)
        
        return calibration_matrix, dist_coef


    def run(self):
        while True:
            ret, frame = self.cap.read()
            frame = cv2.flip(frame, -1)
            corners, ids, _ = aruco.detectMarkers(frame, self.arucoDict)

            if ids is not None and len(ids)>0:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], self.markerSize, self.cameraMatrix, self.distCoef)
                    rvecs, tvecs = rvecs[0], tvecs[0]

                    R = cv2.Rodrigues(rvecs)[0]
                    R_inv = np.linalg.inv(R)
                    camTranslat = np.dot(R_inv, tvecs.reshape(3, 1)).reshape(3)
                    self.carPose.x, self.carPose.y, self.carPose.z = camTranslat[0], camTranslat[1], camTranslat[2]  
                    print(self.carPose.x, self.carPose.y, self.carPose.z)
                    self.pub.publish(self.carPose)
                    self.rate.sleep()

            # cv2.imshow("frame", frame)
            self.cameraClient.sendFrame(frame)
            


arucoloc = Aruco()

arucoloc.run()