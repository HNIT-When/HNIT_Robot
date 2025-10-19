# coding=utf-8
"""现场临时调整杆以及球的HSV阈值化"""

import numpy as np
import time
import cv2
from naoqi import ALProxy
import vision_definitions as vd


class VisualBasis(object):
    """
    a basic class for visual task.
    """

    def __init__(self, robotIp, port=9559, cameraId=vd.kBottomCamera, resolution=vd.kVGA):
        self.cameraProxy = ALProxy("ALVideoDevice", robotIp, port)
        self.motionProxy = ALProxy("ALMotion", robotIp, port)
        self.memoryProxy = ALProxy("ALMemory", robotIp, port)
        self.landmarkProxy = ALProxy("ALLandMarkDetection", robotIp, port)
        self.AutonomousLifeProxy = ALProxy("ALAutonomousLife", robotIp, port)
        self.cameraId = cameraId
        self.cameraName = "CameraBottom" if self.cameraId == vd.kBottomCamera else "CameraTop"
        self.resolution = resolution  # 分辨率
        self.colorSpace = vd.kBGRColorSpace  # 色彩空间
        self.fps = 20  # 帧率
        self.frameHeight = 0
        self.frameWidth = 0
        self.frameChannels = 0
        self.frameArray = None  # 帧数组
        self.cameraPitchRange = 47.64 / 180 * np.pi  # 倾斜度
        self.cameraYawRange = 60.97 / 180 * np.pi  # 偏转度
        self.cameraProxy.setActiveCamera(self.cameraId)  # 设置拍照相机

    def updateFrame(self, client="python_client"):
        """
        get a new Image from the specified camera and save it in self._frame.
        从指定的相机获取新图像，并将其保存在self._frame
        """
        if self.cameraProxy.getActiveCamera() != self.cameraId:
            self.cameraProxy.setActiveCamera(self.cameraId)
            time.sleep(1)

        videoClient = self.cameraProxy.subscribe(client, self.resolution, self.colorSpace, self.fps)  # 已弃用
        frame = self.cameraProxy.getImageRemote(videoClient)  # 远程获取视频源最后拍摄的图像
        self.cameraProxy.unsubscribe(videoClient)
        try:
            self.frameWidth = frame[0]  # 图像宽度
            self.frameHeight = frame[1]
            self.frameChannels = frame[2]  # 图像层数，像素字节数  frame[6]图像数据
            # 用np.frombuffer读进来img后, shape是(47040000..), 在reshape的时候需要把形状变成([frame[1], frame[0], frame[2]])
            self.frameArray = np.frombuffer(frame[6], dtype=np.uint8).reshape([frame[1], frame[0], frame[2]])
        except IndexError:
            print("get Image failed!")


class ObjectDetection(VisualBasis):
    """
    对图像进行预处理和调节的滑动条
    """

    def __init__(self, robotIp, cameraId=vd.kBottomCamera):
        super(ObjectDetection, self).__init__(robotIp, cameraId=cameraId)
        self.updateFrame()  # 获取图像
        self.img = self.frameArray  # 图像数据

    def sliderObjectHSV(self, classfier):  # 红球(redball)/黄杆(stick)
        """
        HSV滑动条函数，为了获得理想的HSV阈值
        """
        if classfier == "redball":
            cv2.namedWindow("redball")
            # 创建滑动条
            cv2.createTrackbar("hmax1", "redball", 10, 20, lambda x: None)  # lambda匿名函数
            cv2.createTrackbar("smin1", "redball", 43, 60, lambda x: None)
            cv2.createTrackbar("vmin1", "redball", 46, 60, lambda x: None)
            cv2.createTrackbar("hmin2", "redball", 156, 175, lambda x: None)
            img = self.img.copy()  # 局部变量
            HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            while True:
                # time.sleep(2.0)
                # 获取滑动条的值
                hmax1 = cv2.getTrackbarPos("hmax1", "redball")
                smin1 = cv2.getTrackbarPos("smin1", "redball")
                vmin1 = cv2.getTrackbarPos("vmin1", "redball")
                hmin2 = cv2.getTrackbarPos("hmin2", "redball")

                # HSV空间颜色判断
                minHSV1 = np.array([0, smin1, vmin1])
                maxHSV1 = np.array([hmax1, 255, 255])
                minHSV2 = np.array([hmin2, smin1, vmin1])
                maxHSV2 = np.array([180, 255, 255])
                binImg1 = cv2.inRange(HSVImg, minHSV1, maxHSV1)  # 设阈值,去除其他部分
                binImg2 = cv2.inRange(HSVImg, minHSV2, maxHSV2)
                binImg = np.maximum(binImg1, binImg2)  # 分别是在两个多维数组中逐元素求最大值

                # 图像滤波处理
                binImg = self.Filter(binImg)

                cv2.imshow("srcImg", img)  # 原始图像
                cv2.imshow("redball", binImg)  # 提取颜色加滤波后的图像
                exitKey = cv2.waitKey(1)
                if exitKey == 27:
                    print("self.hmax1 = {}\nself.smin1 = {}\nself.vmin1 = {}\nself.hmin2 = {}".format(
                        hmax1, smin1, vmin1, hmin2))
                    cv2.destroyAllWindows()
                    break

        elif classfier == "stick":
            cv2.namedWindow("stick")
            # 创建滑动条
            cv2.createTrackbar("hmin", "stick", 27, 30, lambda x: None)
            cv2.createTrackbar("hmax", "stick", 45, 50, lambda x: None)
            cv2.createTrackbar("smin", "stick", 55, 60, lambda x: None)
            cv2.createTrackbar("vmin", "stick", 115, 200, lambda x: None)
            img = self.img.copy()
            HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            while True:
                # time.sleep(2.0)
                # 获取滑动条的值
                self.img = self.frameArray
                hmin = cv2.getTrackbarPos("hmin", "stick")
                hmax = cv2.getTrackbarPos("hmax", "stick")
                smin = cv2.getTrackbarPos("smin", "stick")
                vmin = cv2.getTrackbarPos("vmin", "stick")

                # HSV空间颜色判断
                minHSV = np.array([hmin, smin, vmin])
                maxHSV = np.array([hmax, 255, 255])
                binImg = cv2.inRange(HSVImg, minHSV, maxHSV)

                # 图像滤波处理
                binImg = self.Filter(binImg)

                cv2.imshow("srcImg", img)
                cv2.imshow("stick", binImg)
                exitKey = cv2.waitKey(1)
                if exitKey == 27:
                    print("hmin = {}\nhmax = {}\nsmin = {}\nvmin = {}".format(hmin, hmax, smin, vmin))
                    cv2.destroyAllWindows()
                    break

    def Filter(self, binImg):
        kernelErosion = np.ones((3, 3), np.uint8)
        kernelDilation = np.ones((3, 3), np.uint8)
        resImg = cv2.erode(binImg, kernelErosion, iterations=2)  # 腐蚀
        resImg = cv2.dilate(resImg, kernelDilation, iterations=3)  # 膨胀
        resImg = cv2.GaussianBlur(resImg, (9, 9), 1.5)  # 高斯滤波
        return resImg

    def preprocess(self, img, classfier):  # 预处理
        if classfier == 'redball':
            HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            minHSV1 = np.array([0, 43, 46])
            maxHSV1 = np.array([10, 255, 255])
            minHSV2 = np.array([156, 43, 46])
            maxHSV2 = np.array([180, 255, 255])

            # 二值化处理
            binImg1 = cv2.inRange(HSVImg, minHSV1, maxHSV1)
            binImg2 = cv2.inRange(HSVImg, minHSV2, maxHSV2)
            binImg = np.maximum(binImg1, binImg2)  # 两个比较取最大的值，每个

            resImg = self.Filter(binImg)

            return resImg

        elif classfier == 'stick':
            HSVImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转到HSV空间
            hmin, hmax, smin, vmin = 27, 37, 57, 108
            # 二值化处理
            minHSV = np.array([hmin, smin, vmin])
            maxHSV = np.array([hmax, 255, 255])
            binImg = cv2.inRange(HSVImg, minHSV, maxHSV)

            # 图像滤波处理（腐蚀，膨胀，高斯）
            resImg = self.Filter(binImg)
            return resImg


class RedBallDetection(ObjectDetection):
    def __init__(self, robotIp, cameraId=vd.kBottomCamera):
        super(RedBallDetection, self).__init__(robotIp, cameraId=cameraId)
        self.img = self.frameArray

    def circle2Rect(self, circle, k=1):
        """
        圆的信息转换为矩阵信息，以便后续处理
        Arguments:
            circle：圆的信息：圆心坐标，半径
            k：放缩因子
        Return:
            rect：矩阵信息：左上角和右下角的坐标
        """
        x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
        initX, initY = x - k * r, y - k * r  # 矩形左上角坐标
        endX, endY = x + k * r, y + k * r  # 矩形右下角坐标
        rect = [initX, initY, endX, endY]

        return rect

    def showHoughResult(self, img, circles, timeMs=0):  # 图像 圆 延迟时间
        """
        显示霍夫圆检测结果
        """
        for circle in circles:
            rect = self.circle2Rect(circle)  # 矩阵信息
            initX, initY = rect[0], rect[1]
            endX, endY = rect[2], rect[3]
            cv2.rectangle(img, (initX, initY), (endX, endY), (0, 0, 255), 2)  # 画矩形

            x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
            cv2.circle(img, (x, y), r, (0, 0, 255), 2)  # 画圆

        cv2.imshow("Hough Result", img)
        cv2.waitKey(timeMs)
        cv2.destroyAllWindows()

    def houghDetection(self, isShow=True):
        img = self.img.copy()
        binImg = self.preprocess(img, 'redball')
        circles = cv2.HoughCircles(binImg, cv2.HOUGH_GRADIENT, 1, 100, param1=150, param2=15, minRadius=2, maxRadius=80)
        if circles is None:
            circles = []
            print("no circle")
            return circles
        else:
            circles = circles[0, :]
            if isShow is True:  # 显示轮廓信息
                self.showHoughResult(img, circles)  # 圆 矩形
        # 返回检测到的圆的轮廓矩阵
        return circles


class ContoursDetection(ObjectDetection):  # 轮廓检测
    def __init__(self, robotIp, cameraId=vd.kTopCamera):
        super(ContoursDetection, self).__init__(robotIp, cameraId=cameraId)
        self.img = self.frameArray

    def contoursDetection(self, minPerimeter=100, minArea=850, isShow=True):  # 最小周长面积

        img = self.img
        binImg = self.preprocess(img, 'stick')  # 二值化 腐蚀 膨胀 高斯滤波

        frameBin = self.Filter(binImg)  # 图像滤波处理（腐蚀，膨胀，高斯）

        rects = []
        if cv2.__version__.split(".")[0] == "3":  # for OpenCV >= 3.0.0
            _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找轮廓
        if len(contours) == 0:
            return rects
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)  # 周长
            area = cv2.contourArea(contour)  # 面积
            if perimeter > minPerimeter and area > minArea:
                x, y, w, h = cv2.boundingRect(contour)  # 获得一个图像的最小矩形边框
                rects.append([x, y, w, h])
        if len(rects) is 0:
            return rects
        if isShow is True:
            self.showContourResult(img, contours)
        return rects

    def showContourResult(self, img, contours, timeMs=0):
        """
        显示轮廓检测结果
        Arguments:
            img：图像
            contours：轮廓
            timeMs：延迟时间，0表示一直显示
        """
        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)  # 画轮廓
        for contour in contours:  # 画出轮廓的外接矩形
            rect = self.contour2Rect(contour)
            cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (0, 0, 255), 2)

        cv2.imshow("Contour_result", img)
        cv2.waitKey(timeMs)
        cv2.destroyAllWindows()

    def contour2Rect(self, contour):
        """
        轮廓的信息转换为矩阵信息，以便后续处理
        Arguments:
            contour：轮廓的信息：若干个点组成的轮廓
        Return:
            rect：矩阵信息：左上角和右下角的坐标
        """
        x, y, w, h = cv2.boundingRect(contour)  # 返回值为外接矩阵的顶点坐标和长宽

        rect = [x, y, x + w, y + h]
        return rect

    def contoursSlider(self):
        """
        轮廓检测滑动条，为了获得minPer理想值
        """
        cv2.namedWindow("result")
        cv2.createTrackbar("minPerimeter", "result", 100, 500, lambda x: None)
        cv2.createTrackbar("minArea", "result", 850, 1000, lambda x: None)
        img = self.img.copy()

        while True:
            minPer = cv2.getTrackbarPos("minPerimeter", "result")
            minArea = cv2.getTrackbarPos("minArea", "result")
            self.contoursDetection(img, minPer, minArea)


def main(IP, name):
    if name == 'redball':
        redball = RedBallDetection(IP)
        redball.sliderObjectHSV(name)  # 父类方法滑动条显示
    if name == 'stick':
        stick = ContoursDetection(Ip)  # 轮廓检测 绘制轮廓矩形
        stick.contoursDetection()
        stick.sliderObjectHSV('stick')


if __name__ == '__main__':
    Ip = "192.168.0.103"
    # 球HSV调整
    # main(Ip, 'redball')

    # 杆HSV调整
    main(Ip, 'stick')
# 师大
# 球
# self.hmax1 = 15
# self.smin1 = 42
# self.vmin1 = 34
# self.hmin2 = 154

# 杆(远处)
# hmin = 25
# hmax = 35
# smin = 56
# vmin = 87

# 杆（近处）
# hmin = 29
# hmax = 40
# smin = 53
# vmin = 108

# 长沙大学
# 杆
# hmin = 27
# hmax = 37
# smin = 57
# vmin = 108

# 手机
# hmin = 25
# hmax = 44
# smin = 11
# vmin = 83