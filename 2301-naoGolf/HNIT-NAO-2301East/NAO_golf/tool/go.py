# coding=utf-8
import cv2
import numpy as np
import time
from naoqi import ALProxy
import motion
import almath


def walk_step2():
    walk_step_ = [["MaxStepFrequency", 0.4],
                  ["MaxStepX", 0.045],
                  ["MaxStepY", 0.145],
                  ["MaxStepTheta", 0.20],
                  ["StepHeight", 0.014],
                  ["TorsoWx", 0.0],
                  ["TorsoWy", 0.05]]
    return walk_step_

def walk_step():
    walk_step_ = [["MaxStepFrequency", 0.80],
                  ["MaxStepX", 0.030],
                  ["MaxStepY", 0.13],
                  ["MaxStepTheta", 0.23],
                  ["StepHeight", 0.014],
                  ["TorsoWx", 0.00],
                  ["TorsoWy", 0.00]]
    return walk_step_

IP = "192.168.31.63"
if __name__ == "__main__":
    motionProxy = ALProxy("ALMotion", IP, 9559)
    motionProxy.wakeUp()
    motionProxy.moveInit()
    motionProxy.moveTo(0, -0.5, 0,walk_step())

    # self.walk_to_ball()#
    # self.mark_ball_robot()
    # motionProxy.moveTo(0, 0, 45 , walk_step())
    # motionProxy.moveTo(0, 0, 40 , walk_step())
    # motionProxy.moveTo(-0.10, 0, 0, walk_step())  # 后退
    # motionProxy.moveTo(0, -0.17, 0, walk_step())  # 斜着走
    #motionProxy.rest()

