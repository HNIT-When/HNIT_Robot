# coding=utf-8

import os
import numpy as np
import vision_definitions as vd
import time
import motion
import cv2
import almath
import socket
import shutil


global channelB, channelG, channelR
rad = almath.TO_RAD


def hsv_detection(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_bin = cv2.inRange(frame_hsv, minHsv, maxHsv)
    kernel_erosion = np.ones((5, 5), np.uint8)
    kernel_dilation = np.ones((5, 5), np.uint8)
    frame_bin = cv2.erode(frame_bin, kernel_erosion, iterations=1)  # 腐蚀
    frame_bin = cv2.dilate(frame_bin, kernel_dilation, iterations=1)  # 膨胀
    frame_bin = cv2.GaussianBlur(frame_bin, (3, 3), 0)  # 高斯滤波

    cv2.imshow("image", frame_bin)
    cv2.waitKey(1000)
    contours, _ = cv2.findContours(frame_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours


def draw_contour(contours, frame):
    # rects = []  # 选择合适的方框
    # for contour in contours:  # 找出合格的轮廓
    #     perimeter = cv2.arcLength(contour, True)
    #     print(perimeter)
    #     if perimeter <= contourRange[0] or perimeter >= contourRange[1]:  # 对轮廓周长限制
    #         print "findYellowStick(): to large or to small"
    #         continue  # 终止本次循环
    #     w = cv2.boundingRect(contour)[2]
    #     if w > 100:  # 对宽进行限制
    #         print("wide is not qualified")
    #         continue
    #     rects.append(contour)
    # # if len(rects) == 0:
    # #     return np.array([])
    # contours = rects

    max_contour = contours[0]
    max_perimeter = contourRange[0]
    for contour in contours:  # 找出最大的轮廓
        perimeter = cv2.arcLength(contour, True)
        if perimeter > max_perimeter:
            max_perimeter = perimeter
            max_contour = contour
    [x, y, w, h] = cv2.boundingRect(max_contour)

    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.imshow("stick position", frame)
    cv2.waitKey(100000)


def preprocess(frame_array, min_hsv, max_hsv):
    frame_hsv = cv2.cvtColor(frame_array, cv2.COLOR_BGR2HSV)
    frame = cv2.inRange(frame_hsv, min_hsv, max_hsv)
    kernel_erosion = np.ones((5, 5), np.uint8)
    kernel_dilation = np.ones((5, 5), np.uint8)
    frame = cv2.erode(frame, kernel_erosion, iterations=1)  # 腐蚀
    frame = cv2.dilate(frame, kernel_dilation, iterations=1)  # 膨胀
    frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯滤波
    return frame


def slider(frame):
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

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_bin = cv2.inRange(frame_hsv, min_hsv, max_hsv)
        kernel_erosion = np.ones((5, 5), np.uint8)
        kernel_dilation = np.ones((5, 5), np.uint8)
        frame_bin = cv2.erode(frame_bin, kernel_erosion, iterations=1)  # 腐蚀
        frame_bin = cv2.dilate(frame_bin, kernel_dilation, iterations=1)  # 膨胀
        frame_bin = cv2.GaussianBlur(frame_bin, (3, 3), 0)  # 高斯滤波

        cv2.namedWindow("frame_bin", 0)
        cv2.resizeWindow("frame_bin", 800, 400)
        cv2.imshow("frame_bin", frame_bin)
        if cv2.waitKey(10) & 0xFF == 27:
            break

if __name__ == "__main__":
    # frame = cv2.imread("D:\\PyCharm\\Nao-golf\\NAO_golf_V3.0\\tool\\yellow_stick.jpg")
    frame = cv2.imread("../save_pictures/stick_HSV_test/1.jpg")
    # frameWidth = frame[0]
    # frameHeight = frame[1]
    # frameChannels = frame[2]
    # frame = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1], frame[0], frame[2]])
    minHsv = np.array([21, 100, 60])  # [30, 55, 70] [28, 100, 93] [
    maxHsv = np.array([44, 255, 255])  # [45, 255, 255] [42, 255, 255]
    contourRange = [120, 650]  # 实测最大450左右

    # slider(frame)
    contours = hsv_detection(frame)
    draw_contour(contours, frame)
