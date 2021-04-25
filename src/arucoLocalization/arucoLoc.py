import cv2
from cv2 import aruco
import numpy as np
from math import cos, sin, atan, atan2
from utils import *


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

            corners, ids, _ = aruco.detectMarkers(frame, self.arucoDict)

            if ids is not None and len(ids)>0:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], self.markerSize, self.cameraMatrix, self.distCoef)
                    print(rvecs, tvecs)
            # cv2.imshow("frame", frame)
            self.cameraClient.sendFrame(frame)
            


arucoloc = Aruco()

arucoloc.run()