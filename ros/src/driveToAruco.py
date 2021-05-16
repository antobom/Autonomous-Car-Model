#!/usr/bin/env python3

import rospy
import numpy as np
from utils import CarControl
from utils import Position, Orientation
from autonomous_car_model.msg import CarPose

def callback(carPose:CarPose):
    carPosition = Position(carPose.x, carPose.y, carPose.z, carPose.rx, carPose.ry, carPose.rz)
    targetPosition = Position()
    carControl.driveToTarget(targetPosition, carPosition)


def run():
    rospy.init_node("driveToAruco")

    rospy.Subscriber("car_position", CarPose, callback = callback)
    rospy.spin()

carControl = CarControl()
if __name__=="__main__":
    print("running")
    run()
