# coding=utf-8
import cv2
import numpy as np
import time
from naoqi import ALProxy
import vision_definitions as vd
import motion
import math
import os
import sys
import time
import threading  # 线程模块


def walk_slow():
    walk_slow = [["MaxStepFrequency", 0.80],
                 ["MaxStepX", 0.035],
                 ["MaxStepY", 0.11],
                 ["MaxStepTheta", 0.22],
                 ["StepHeight", 0.0135],
                 ["TorsoWx", 0.00],
                 ["TorsoWy", 0.00]
                 ]
    return walk_slow


class ConfigureNao(object):
    def __init__(self, IP, PORT=9559):
        self.IP = IP
        self.PORT = PORT
        try:
            self.cameraProxy = ALProxy("ALVideoDevice", self.IP, self.PORT)
            self.motionProxy = ALProxy("ALMotion", self.IP, self.PORT)
            self.postureProxy = ALProxy("ALRobotPosture", self.IP, self.PORT)  # 预定义姿势
            self.tts = ALProxy("ALTextToSpeech", self.IP, self.PORT)
            self.memoryProxy = ALProxy("ALMemory", self.IP, self.PORT)
            self.landMarkProxy = ALProxy("ALLandMarkDetection", self.IP, self.PORT)
            self.ALAutonomousLifeProxy = ALProxy("ALAutonomousLife", self.IP, self.PORT)  # 自主生活模式
        except Exception as e:
            print("Error when configuring the NAO!")
            print(str(e))
            exit(1)


class VisualBasis(ConfigureNao):
    def __init__(self, IP, PORT=9559, cameraId=vd.kTopCamera, resolution=vd.kVGA):
        super(VisualBasis, self).__init__(IP, PORT)
        self.cameraId = cameraId
        self.cameraName = "CameraBottom" if self.cameraId == vd.kBottomCamera else "CameraTop"
        self.resolution = resolution
        self.colorSpace = vd.kBGRColorSpace
        self.fps = 20
        self.frameHeight = 0
        self.frameWidth = 0
        self.frameChannels = 0
        self.frameArray = None
        self.cameraPitchRange = 47.64 / 180 * np.pi
        self.cameraYawRange = 60.97 / 180 * np.pi
        self.cameraProxy.setActiveCamera(self.cameraId)
        self.cameraProxy.unsubscribe("python_client_1")

    def updateFrame(self, client="python_client"):
        if self.cameraProxy.getActiveCamera() != self.cameraId:
            self.cameraProxy.setActiveCamera(self.cameraId)

        videoClient = self.cameraProxy.subscribe(client, self.resolution, self.colorSpace, self.fps)  # 以弃用
        frame = self.cameraProxy.getImageRemote(videoClient)  # 获取视频源最后的图像
        self.cameraProxy.unsubscribe(videoClient)
        try:
            self.frameWidth = frame[0]
            self.frameHeight = frame[1]
            self.frameChannels = frame[2]
            self.frameArray = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1], frame[0], frame[2]])
        except IndexError:
            print("get Image failed!")


class LandMarkDetect(VisualBasis):
    def __init__(self, IP, PORT=9559, cameraId=vd.kTopCamera, resolution=vd.kVGA,
                 writeFrame=False, writeFramewithCircle=False):
        super(LandMarkDetect, self).__init__(IP, PORT, cameraId, resolution)
        self.writeFrame = writeFrame
        self.writeFramewithCircle = writeFramewithCircle

    def start(self):
        self.motionProxy.wakeUp()
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)
        self.motionProxy.moveInit()
        # moveTask_1 = self.motionProxy.post.moveTo(10, 0, 0, walk_slow())  # 新进程
        for i in range(100):
            self.updateFrame()
            img = self.frameArray
            # 显示
            # cv2.imshow('window_title', img)
            # cv2.waitKey(1000)
            # cv2.destroyAllWindows()
            # 保存
            print("save %d pictures" % i)
            img_path = "../save_pictures/mark_pictures_310/" + str(i) + ".jpg"
            cv2.imwrite(img_path, img)
        # self.motionProxy.stop(moveTask_1)


if __name__ == "__main__":
    ball = LandMarkDetect("192.168.31.63")
    ball.start()
