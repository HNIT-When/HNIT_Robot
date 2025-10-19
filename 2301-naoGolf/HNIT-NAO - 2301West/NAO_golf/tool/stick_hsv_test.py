# coding=utf-8
"""
拍摄不同角度距离的六张黄杆图片，找到合适的hsv值
"""

import cv2
import time
import numpy as np
from naoqi import ALProxy
# import matplotlib.pyplot as plt
import vision_definitions as vd


def take_picture():
    for i in range(6):
        while True:
            front_flag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
            middle_flag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
            rear_flag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
            if front_flag or middle_flag or rear_flag:
                led.off("FaceLeds")
                break
        video_client = cameraProxy.subscribeCamera("video_client", vd.kTopCamera, vd.kVGA, vd.kBGRColorSpace, 20)
        frame = cameraProxy.getImageRemote(video_client)
        cameraProxy.unsubscribe(video_client)
        frame_array = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1], frame[0], frame[2]])
        img[i] = frame_array
        cv2.imwrite("../save_pictures/stick_HSV_test/" + str(i) + ".jpg", frame_array)
        led.on("FaceLeds")


def preprocess(frame_array, min_hsv, max_hsv):
    frame_hsv = cv2.cvtColor(frame_array, cv2.COLOR_BGR2HSV)
    frame = cv2.inRange(frame_hsv, min_hsv, max_hsv)
    kernel_erosion = np.ones((5, 5), np.uint8)
    kernel_dilation = np.ones((5, 5), np.uint8)
    frame = cv2.erode(frame, kernel_erosion, iterations=1)  # 腐蚀
    frame = cv2.dilate(frame, kernel_dilation, iterations=1)  # 膨胀
    frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯滤波
    return frame


def slider():
    """滑动条用于在HSV颜色空间中检测棍子"""
    window_name = "slider for stick detection"
    cv2.namedWindow(window_name)
    cv2.resizeWindow(window_name, 640, 300)
    cv2.createTrackbar("minH", window_name, 20, 100, lambda x: None)
    cv2.createTrackbar("minS", window_name, 20, 100, lambda x: None)
    cv2.createTrackbar("minV", window_name, 60, 160, lambda x: None)
    cv2.createTrackbar("maxH", window_name, 20, 100, lambda x: None)
    while True:
        min_h = cv2.getTrackbarPos("minH", window_name)
        min_s = cv2.getTrackbarPos("minS", window_name)
        min_v = cv2.getTrackbarPos("minV", window_name)
        max_h = cv2.getTrackbarPos("maxH", window_name)
        min_hsv = np.array([min_h, min_s, min_v])
        max_hsv = np.array([max_h, 255, 255])
        for i in range(6):
            frame_bin[i] = preprocess(img[i], min_hsv, max_hsv)
        frame_bin_h1 = np.hstack((frame_bin[0], frame_bin[1], frame_bin[2]))
        frame_bin_h2 = np.hstack((frame_bin[3], frame_bin[4], frame_bin[5]))
        frame_bins = np.vstack((frame_bin_h1, frame_bin_h2))
        cv2.namedWindow("frame_bin", 0)
        cv2.resizeWindow("frame_bin", 800, 400)
        cv2.imshow("frame_bin", frame_bins)
        if cv2.waitKey(10) & 0xFF == 27:
            break


IP = "192.168.206.175"
port = 9559
motion = ALProxy("ALMotion", IP, port)
tts = ALProxy("ALTextToSpeech", IP, port)
memoryProxy = ALProxy("ALMemory", IP, port)
posture = ALProxy("ALRobotPosture", IP, port)
cameraProxy = ALProxy("ALVideoDevice", IP, port)
led = ALProxy("ALLeds", IP, port)

img = {}
frame_bin = {}

if __name__ == "__main__":
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.6)
    tts.say("拍照开始")
    take_picture()
    img_h1 = np.hstack((img[0], img[1], img[2]))
    img_h2 = np.hstack((img[3], img[4], img[5]))
    imgs = np.vstack((img_h1, img_h2))
    cv2.namedWindow("stick", 0)
    cv2.resizeWindow("stick", 800, 400)
    cv2.imshow("stick", imgs)
    tts.say("结束")
    motion.rest()
    slider()
    cv2.destroyAllWindows()
