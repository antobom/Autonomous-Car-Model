import cv2
import pickle
import numpy as np
import socket


class CameraServer:
    def __init__(self):
        self.HOST = "192.168.179.21"
        self.PORT = 65432

        self.socket_server = socket.socket()
        self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


    def waitForConnection(self):
        self.socket_server.bind((self.HOST, self.PORT))
        self.socket_server.listen(2)
        self.conn, adrr = self.socket_server.accept()

        print("{} is connected".format(adrr))


    def recvAll(self, size, conn):
        data = b""
        while len(data) <size:
            data+=conn.recv(4096)
        return pickle.loads(data)

    def confirm(self):
        self.conn.send(b"1")
    def kill(self):
        self.conn.send(b"0")
        
    def recieveImage(self):
        size = pickle.loads(self.conn.recv(4096))
        self.confirm()
        data = self.recvAll(size, self.conn)
        self.confirm()
        frame = cv2.imdecode(data, 1)
        return frame

    def displayImage(self, image):
        # image = cv2.flip(image, -1)
        cv2.imshow("frame", image)

        return cv2.waitKey(1) != ord('q')
        

            


if __name__ == "__main__":
    cameraServer = CameraServer()
    cameraServer.waitForConnection()
    while True:
        frame = cameraServer.recieveImage()
        if not cameraServer.displayImage(frame):
            break








