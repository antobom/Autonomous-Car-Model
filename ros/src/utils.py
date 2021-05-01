import cv2
import numpy as np
import pickle
import socket




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


if __name__ == "__main__":
    cameraClient = CameraClient()
    cameraClient.connect()
    while True:
        frame = cameraClient.getFrame()
        cameraClient.sendFrame(frame)

