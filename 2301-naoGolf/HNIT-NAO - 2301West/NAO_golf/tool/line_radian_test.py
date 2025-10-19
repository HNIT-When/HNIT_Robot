# coding=utf-8
from naoqi import ALProxy
import time
import numpy as np
import almath
from visualTaskGai import WhiteLine
import vision_definitions as vd
import cv2


IP = "192.168.11.26"  # 机器人的IP地址
PORT = 9559  # 机器人的端口号，默认9559
rad = almath.TO_RAD  # 1度所对应的弧度数 0.0174532923847


def walk_rotate():
    walk_slow = [["MaxStepFrequency", 0.98],
                 ["MaxStepX", 0.035],
                 ["MaxStepY", 0.11],
                 ["MaxStepTheta", 0.05],
                 ["StepHeight", 0.0135],
                 ["TorsoWx", 0.00],
                 ["TorsoWy", 0.00]
                 ]
    return walk_slow


motion = ALProxy("ALMotion", IP, PORT)

whiteLine = WhiteLine(IP, camera_id=vd.kBottomCamera)  # 白线检测


def line_robot(deviation=0.0):  # 想要机器人向右传负值，左边传正值
    """机器人平行白线"""
    for i in range(10):
        whiteLine.update_line_data()
        whiteLine.show_line()
        whiteLine.lineRadian += deviation  # 第三关向右偏移,弧度
        print ("angle: {}".format(whiteLine.lineRadian))
        if abs(whiteLine.lineRadian) < 0.015 and len(whiteLine.lineRect):
            break
        motion.moveTo(0, 0, whiteLine.lineRadian, walk_rotate())


def line_test():
    frame = cv2.imread("./a.jpg")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src_blur = cv2.GaussianBlur(gray, (3, 3), 0)
    kernel_erosion = np.ones((3, 3), np.uint8)
    kernel_dilation = np.ones((3, 3), np.uint8)
    gray = cv2.erode(src_blur, kernel_erosion, iterations=1)  # 腐蚀
    gray = cv2.dilate(src_blur, kernel_dilation, iterations=3)  # 膨胀

    cv2.imshow("test", gray)
    cv2.waitKey(1)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    cv2.imshow("edges", edges)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 60)

    lineRect = lines[0][0]
    rho = lineRect[0]  # 像素长度
    theta = lineRect[1]  # 弧度
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * a)
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * a)
    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv2.line(frame, (0, 240), (480, 240), (255, 0, 0), 2)  # 屏幕中心画线
    cv2.imshow("line", frame)
    cv2.waitKey(1)


time.sleep(1)
motion.wakeUp()
motion.moveInit()
motion.setStiffnesses("Body", 1.0)
motion.setMoveArmsEnabled(False, False)  # 手臂不动
motion.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
motion.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)

if __name__ == "__main__":
    # while True:
    #     line_test()
    line_robot(-0.0)

    motion.rest()
