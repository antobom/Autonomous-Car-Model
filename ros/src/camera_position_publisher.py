#!/usr/bin/env python3

from os import chdir
import cv2
from cv2 import aruco
import numpy as np
from math import cos, sin, atan, atan2, asin, acos, sqrt
from utils import *
# from autonomous_car_model.srv import car_pose
from autonomous_car_model.msg import CarPose, CarOrientation
import rospy

class Aruco:
    def __init__(self):
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL )
        self.arucoParam = aruco.DetectorParameters_create()
        self.markerSize = 0.085
       
        self.cap = cv2.VideoCapture(0)

        self.cameraMatrix, self.distCoef = self.loadCalibrationMatrixCoef()


        #test
        self.connectToServer = False
        if self.connectToServer:
            self.cameraClient = CameraClient()
            self.cameraClient.connect()

        rospy.init_node("aruco_camera")
        self.pubPosition = rospy.Publisher("car_position", CarPose, queue_size = 10)
        # self.pubOrientation = rospy.Publisher("car_orientation", CarOrientation, queue_size = 10)
        
        self.rate = rospy.Rate(5)
        self.carPose = CarPose()
        # self.carOrientation = CarOrientation()

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

    def rotationMatrixToEuler(self, R):
        #http://www.close-range.com/docs/Computing_Euler_angles_from_a_rotation_matrix.pdf
        
        sy = sqrt(R[0, 0]**2 + R[1, 0]**2)
        if not sy<1e-6:
            rx = atan2(R[2, 1], R[2, 2])
            ry  = atan2(-R[2, 0], sy)
            rz  = atan2(R[1, 0], R[0, 0])
        else:           
            rx = atan2(-R[1, 2], R[1, 1])
            ry  = atan2(-R[2, 0], sy)
            rz  = 0

        return rx, ry, rz

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

                    extrinsics = np.zeros((4, 4))
                    R = cv2.Rodrigues(rvecs)[0]
                    R_inv = np.linalg.inv(R)

                    self.carPose.x, self.carPose.y, self.carPose.z = np.dot(R_inv, tvecs.reshape(3, 1)).reshape(3)
                    self.carPose.rx, self.carPose.ry, self.carPose.rz = self.rotationMatrixToEuler(R_inv)
                    # self.carOrientation.x, self.carOrientation.y, self.carOrientation.z = self.rotationMatrixToEuler(R_inv)
                    
                    self.pubPosition.publish(self.carPose)
                    # self.pubOrientation.publish (self.carOrientation)

            self.rate.sleep()

            # cv2.imshow("frame", frame)
            if self.connectToServer:
                self.cameraClient.sendFrame(frame)
            

chdir("/home/pi/Documents/ros_ws/src/autonomous_car_model/src")

arucoloc = Aruco()

arucoloc.run()