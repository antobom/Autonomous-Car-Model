import cv2
import numpy as np
import pickle
import socket
from math import asin, acos, sin, cos, atan2
import serial
import time

class Position:
    def __init__(self, x = 0, y = 0, z =0, rx=0, ry=0, rz=0):
        self.x = x
        self.y = y
        self.z = z
        self.orientation = Orientation(rx, ry, rz)
    
    def __add__(self, other):
        assert(type(other) == type(Position))

        x = self.x + other.x
        y = self.z + other.z
        z = self.z + other.z
        return Position(x, y, z)

class Orientation:
    def __init__(self, rx=0, ry=0, rz=0):
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def getRotationMatrix(self):
        pass
    def getRy(self):
        Ry = np.array([ [cos(self.ry), -sin(self.ry)],
                        [sin(self.ry),  cos(self.ry)] ])
        return Ry

class CameraClient:
    def __init__(self):

        self.HOST = "192.168.179.21"
        self.PORT = 65432
        self.socketClient = socket.socket()

        # self.cap = cv2.VideoCapture(0)


    def wait_for_response(self):
        response = ''
        while response!='1':
            response = self.socketClient.recv(4096).decode()


    def connect(self):
        self.socketClient.connect((self.HOST, self.PORT))

    def sendFrame(self, frame):
        encodedFrame, frame = cv2.imencode(".jpg", frame)

        data = pickle.dumps(frame)
        size = pickle.dumps(len(data))

        self.socketClient.send(size)
        self.wait_for_response()

        self.socketClient.sendall(data)
        self.wait_for_response()


        return True

    def getFrame(self):
        _, frame = self.cap.read()
        return frame


class CarControl:
    def __init__(self):
        self.serialArduino = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        self.serialArduino.flush()

    def driveToTarget(self, targetPosition:Position, carPosition:Position):
        R = carPosition.orientation.getRy()
        pose_vector = np.array([    [targetPosition.x - carPosition.x],
                                    [targetPosition.y - carPosition.y] ])
        pose_vector = np.dot(np.linalg.inv(R), pose_vector)
        angle = atan2(pose_vector[1], pose_vector[0])
        angle = int(100*(targetPosition.x - carPosition.x))
        #to change maybe add a gain

        self.command_car(angle)
    
    def command_car(self, steeringAngle = None, speed = None):

        if steeringAngle is not None and type(steeringAngle) == int:
            steeringAngle = steeringAngle/abs(steeringAngle) * min(abs(steeringAngle), 30)
            steeringAngle =int(90 + steeringAngle)
            line = "ser" + str(steeringAngle) + "\n"
            self.serialArduino.write(line.encode())
            print(line)

        if speed is not None and type(speed) == int:
            time.sleep(0.1)
            line = "mot" + str(speed) + "\n"
            self.serialArduino.write(line.encode())



if __name__ == "__main__":
    cameraClient = CameraClient()
    cameraClient.connect()
    while True:
        frame = cameraClient.getFrame()
        cameraClient.sendFrame(frame)

