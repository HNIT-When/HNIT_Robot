from naoqi import ALProxy
import os
import numpy as np
import vision_definitions as vd
import time
from ConfigureNao import ConfigureNao
import motion
import cv2
import almath
import socket
import shutil
from step import walk_step

global channelB, channelG, channelR
rad = almath.TO_RAD


class VisualBasis(ConfigureNao):
    def __init__(self, robot_ip, port=9559, camera_id=vd.kBottomCamera):
        super(VisualBasis, self).__init__(robot_ip, port)
        self.cameraId = camera_id
        self.resolution = vd.kVGA
        self.colorSpace = vd.kBGRColorSpace
        self.fps = 20
        self.frameHeight = 0
        self.frameWidth = 0
        self.frameChannels = 0
        self.frameArray = None
        self.cameraProxy.unsubscribe("video_client_1")

    def update_frame(self):
        video_client = self.cameraProxy.subscribeCamera("video_client", self.cameraId, self.resolution, self.colorSpace,
                                                        self.fps)
        frame = self.cameraProxy.getImageRemote(video_client)
        self.cameraProxy.unsubscribe(video_client)
        try:
            self.frameWidth = frame[0]
            self.frameHeight = frame[1]
            self.frameChannels = frame[2]
            self.frameArray = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1], frame[0], frame[2]])
        except IndexError:
            print("get Image failed!")

    def get_frame_array(self):
        if self.frameArray is None:
            return np.array([])
        return self.frameArray


IP = "192.168.1.107"
save_img = False
cameraProxy = ALProxy("ALVideoDevice", IP, 9559)

if __name__ == "__main__":
    visual = VisualBasis(IP)
    visual.update_frame()
    frame = visual.get_frame_array()
    print(frame)
    if frame is not None:
        if save_img:
            cv2.imwrite("./frameArray.jpg", frame)
        else:
            cv2.imshow("current frame", frame)
            while True:
                if cv2.waitKey(10) & 0xFF == 27:
                    break
    else:
        print("please get an Image from Nao with the method updateFrame()")



