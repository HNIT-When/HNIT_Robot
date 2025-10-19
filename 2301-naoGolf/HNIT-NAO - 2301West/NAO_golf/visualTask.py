# coding: utf-8
"""
视觉检测任务(不使用分类器)
"""
import numpy as np
import vision_definitions as vd
from ConfigureNao import ConfigureNao
import motion
import cv2
import almath
import urllib
import json
from step import walk_step

global channelB, channelG, channelR
rad = almath.TO_RAD  # 1度所对应的弧度数 0.0174532923847


class VisualBasis(ConfigureNao):  # 视觉基类
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
        self.cameraPitchRange = 47.64 / 180 * np.pi  # Y
        self.cameraYawRange = 60.97 / 180 * np.pi  # X
        self.cameraProxy.unsubscribe("video_client_1")

    def update_frame(self):
        """更新拍摄到的图片"""
        # self.cameraProxy.setActiveCamera(self.cameraId)
        # video_client = self.cameraProxy.subscribe(client, self.resolution, self.colorSpace, self.fps)  # 以弃用
        video_client = self.cameraProxy.subscribeCamera("video_client", self.cameraId, self.resolution, self.colorSpace,
                                                        self.fps)
        frame = self.cameraProxy.getImageRemote(video_client)  # 获取视频源最后的图像
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

    def show_frame(self, save_img=False):
        if self.frameArray is None:
            print("please get an Image from Nao with the method updateFrame()")
        else:
            cv2.imshow("current frame", self.frameArray)
            if save_img:
                # noinspection PyBroadException
                try:
                    cv2.imwrite("./save_pictures/frameArray.jpg", self.frameArray)
                except Exception:
                    print("Error when saving current frame!")


class BallDetect(VisualBasis):
    def __init__(self, robot_ip, port=9559, camera_id=vd.kBottomCamera):
        super(BallDetect, self).__init__(robot_ip, port, camera_id)
        self.ballData = {"centerX": 0, "centerY": 0, "radius": 0}
        self.ballPosition = {"disX": 0, "disY": 0, "angle": 0}
        self.ballRadius = 0.022

    def update_ball_data(self):
        """更新球的数据"""
        self.update_frame()
        min_dist = int(self.frameHeight / 30.0)
        min_radius = 3
        max_radius = int(self.frameHeight / 10.0)
        gray_frame = self.get_channel_and_blur()  # BGR
        # cv2.imshow("gray_frame", gray_frame)
        circles = self.find_circles(gray_frame, min_dist, min_radius, max_radius)
        circle = self.select_circle(circles)
        if len(circle) == 0:
            self.ballData = {"centerX": 0, "centerY": 0, "radius": 0}
            self.ballPosition = {"disX": 0, "disY": 0, "angle": 0}
        else:
            self.ballData = {"centerX": circle[0], "centerY": circle[1], "radius": circle[2]}
            self.update_ball_position()

    def get_channel_and_blur(self):
        global channelB, channelG, channelR
        # noinspection PyBroadException
        try:
            channelB = self.frameArray[:, :, 0]
            channelG = self.frameArray[:, :, 1]
            channelR = self.frameArray[:, :, 2]
        except Exception:
            print("no Image detected!")
        m = 12  # 调阈值强度，之前是6
        channelB = channelB * 0.1 * m
        channelG = channelG * 0.1 * m
        channelR = channelR - channelB - channelG
        channelR = 3 * channelR
        channelR = cv2.GaussianBlur(channelR, (9, 9), 1.5)  # 高斯滤波
        channelR[channelR < 0] = 0
        channelR[channelR > 255] = 255
        return np.uint8(np.round(channelR))

    @staticmethod
    def find_circles(img, min_dist, min_radius, max_radius):
        gradient_name = cv2.HOUGH_GRADIENT
        circles = cv2.HoughCircles(np.uint8(img), gradient_name, 1,
                                   min_dist, param1=150, param2=20,
                                   minRadius=min_radius, maxRadius=max_radius)
        if circles is None:
            return np.array(np.uint16([]))
        else:
            return np.array(np.uint16(np.around(circles[0])))

    def select_circle(self, circles):
        """"对于每一个检测出的红球，以红球圆心为中心，以红球的4倍半径为边长画一个外围正方形，计算外接正方形区域内红色和绿色像素点所占的比值"""
        if circles.shape[0] == 0:  # circles的行数为0就是没有一个圆
            return circles
        if circles.shape[0] == 1:
            center_x = circles[0][0]
            center_y = circles[0][1]
            radius = circles[0][2]
            init_x = center_x - 2 * radius
            init_y = center_y - 2 * radius
            if (init_x < 0 or init_y < 0 or (init_x + 4 * radius) > self.frameWidth or
                    (init_y + 4 * radius) > self.frameHeight or radius < 1):
                return circles[0]
        r_ratio_min = 1.0
        circle_selected = np.uint16([])
        for circle in circles:
            center_x = circle[0]
            center_y = circle[1]
            radius = circle[2]
            init_x = center_x - 2 * radius
            init_y = center_y - 2 * radius
            if init_x < 0 or init_y < 0 or (init_x + 4 * radius) > self.frameWidth or \
                    (init_y + 4 * radius) > self.frameHeight or radius < 1:
                continue
            rect_ball_area = self.frameArray[init_y:init_y + 4 * radius + 1, init_x:init_x + 4 * radius + 1, :]
            b_flat = np.float16(rect_ball_area[:, :, 0].flatten())  # flatten()是对多维数据的降维函数
            g_flat = np.float16(rect_ball_area[:, :, 1].flatten())
            r_flat = np.float16(rect_ball_area[:, :, 2].flatten())
            r_score1 = np.uint8(r_flat > 1.0 * g_flat)
            r_score2 = np.uint8(r_flat > 1.0 * b_flat)
            r_score = float(np.sum(r_score1 * r_score2))
            g_score = float(np.sum(np.uint8(g_flat > 1.0 * r_flat)))
            r_ratio = r_score / len(r_flat)
            g_ratio = g_score / len(g_flat)
            if r_ratio >= 0.1 and g_ratio >= 0.1 and abs(r_ratio - 0.19) < abs(r_ratio_min - 0.19):
                circle_selected = circle
                r_ratio_min = r_ratio
        return circle_selected

    def update_ball_position(self):  # StandInit:49.2, Stand:39.7
        """更新球的位置 按moveInit状态计算"""
        bottom_camera_direction = {vd.kBottomCamera: 39.7, vd.kTopCamera: 1.2}  # 49.2 39.7 # 45.3, 46.6
        ball_radius = self.ballRadius
        camera_direction = bottom_camera_direction[self.cameraId] * rad  # 摄像头和水平线的夹角弧度
        if self.ballData["radius"] == 0:
            self.ballPosition = {"disX": 0, "disY": 0, "angle": 0}
        else:
            center_x = self.ballData["centerX"]
            center_y = self.ballData["centerY"]
            camera_position = self.motionProxy.getPosition("CameraBottom", motion.FRAME_ROBOT, True)
            camera_x = camera_position[0]  # - 0.027  # 确定重心
            camera_y = camera_position[1]
            camera_height = camera_position[2]
            head_pitches = self.motionProxy.getAngles("HeadPitch", True)  # 返回传感器角度
            head_pitch = head_pitches[0]
            head_yaws = self.motionProxy.getAngles("HeadYaw", True)
            head_yaw = head_yaws[0]
            ball_pitch = (center_y - 240.0) * self.cameraPitchRange / 480.0  # 向下为正
            ball_yaw = (320.0 - center_x) * self.cameraYawRange / 640.0  # 向左为正
            dis_camera = (camera_height - ball_radius) / np.sin(camera_direction + head_pitch + ball_pitch)
            dis_x = (camera_height - ball_radius) / np.tan(camera_direction + head_pitch + ball_pitch)  # SA
            dis_y = dis_camera * np.tan(ball_yaw)
            dis = np.sqrt(dis_x ** 2 + dis_y ** 2)  # 摄像头垂直到水平面的点到红球的距离
            ball_yaw = np.arctan2(dis_y, dis_x)
            ball_x = dis * np.cos(ball_yaw + head_yaw) + camera_x
            ball_y = dis * np.sin(ball_yaw + head_yaw) + camera_y
            ball_yaw = np.arctan2(ball_y, ball_x)
            self.ballPosition["disX"] = ball_x
            self.ballPosition["disY"] = ball_y
            self.ballPosition["angle"] = ball_yaw

    def get_ball_position(self):
        """返回球的位置"""
        dis_x = self.ballPosition["disX"]
        dis_y = self.ballPosition["disY"]
        angle = self.ballPosition["angle"]
        return [dis_x, dis_y, angle]

    def get_ball_image_information(self):
        """返回球在图像中的信息"""
        center_x = self.ballData["centerX"]
        center_y = self.ballData["centerY"]
        radius = self.ballData["radius"]
        return [center_x, center_y, radius]

    def show_ball_position(self, save_img=True):
        """展示图片和输出球的信息,保存图片"""
        cv2.destroyAllWindows()
        if self.ballData["radius"] == 0:
            # print("disX = {}, disY = {}".format(self.ballPosition["disX"], self.ballPosition["disY"]))
            print(self.ballPosition)
            cv2.imshow("ball position", self.frameArray)
            cv2.waitKey(1)
        else:
            # print("ballX = {}, ballY = {}".format(self.ballData["centerX"], self.ballData["centerY"]))
            # print("disX = {}, disY = {}".format(self.ballPosition["disX"], self.ballPosition["disY"]))
            # print("ball direction = {}".format(self.ballPosition["angle"] * 180 / 3.14))
            print(self.ballPosition)
            frame_array = self.frameArray.copy()
            cv2.circle(frame_array, (self.ballData["centerX"], self.ballData["centerY"]),
                       self.ballData["radius"], (250, 150, 150), 2)
            cv2.circle(frame_array, (self.ballData["centerX"], self.ballData["centerY"]),
                       2, (50, 250, 50), 3)
            cv2.imshow("ball position", frame_array)
            cv2.waitKey(1)
            if save_img:  # 保存图片
                # noinspection PyBroadException
                try:
                    cv2.imwrite("./save_pictures/red_ball.jpg", frame_array)
                except Exception:
                    print("Error when saving current frame!")


class StickDetect(VisualBasis):
    def __init__(self, robot_ip, port=9559, camera_id=vd.kTopCamera):
        super(StickDetect, self).__init__(robot_ip, port, camera_id)
        self.boundRect = []
        self.crop = 1  # 0.75 有些时候需要对图片裁剪
        self.stickAngle = 0.0  # rad
        self.contourRange = [100, 800]  # 实测最大450左右
        self.minHsv = np.array([20, 100, 0])  # [20, 100, 40]
        self.maxHsv = np.array([50, 255, 255])  # [44, 255, 255]
        self.stickDistance = 0

    def preprocess(self):
        """对当前帧进行预处理(二值化、裁剪等)，以进行黄杆检测"""
        frame_array = self.frameArray
        try:
            frame_array = frame_array[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪
        except IndexError:
            print("error happened when crop the Image!")
        frame_hsv = cv2.cvtColor(frame_array, cv2.COLOR_BGR2HSV)
        frame_bin = cv2.inRange(frame_hsv, self.minHsv, self.maxHsv)
        kernel_erosion = np.ones((5, 5), np.uint8)
        kernel_dilation = np.ones((5, 5), np.uint8)
        frame_bin = cv2.erode(frame_bin, kernel_erosion, iterations=1)  # 腐蚀
        frame_bin = cv2.dilate(frame_bin, kernel_dilation, iterations=1)  # 膨胀
        frame_bin = cv2.GaussianBlur(frame_bin, (3, 3), 0)  # 高斯滤波
        return frame_bin

    def update_stick_data(self):
        """从指定摄像机更新黄杆数据"""
        self.update_frame()
        frame_bin = self.preprocess()
        contours = self.find_contours(frame_bin)
        contour = self.select_contour(contours)
        if len(contour) == 0:  # rect为空
            self.boundRect = []
            self.stickAngle = 0.0
        else:
            print("rect:{}".format(contour))
            self.boundRect = contour
            self.get_stick_angle(contour)

    @staticmethod
    def find_contours(frame_bin):
        """在预处理的帧中找到黄色的条"""
        #版本问题 参数个数@TODO
        contours, _ = cv2.findContours(frame_bin.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours is None:
            return np.array([])
        else:
            return contours

    def select_contour(self, contours):
        """找出最适合的轮廓"""
        if not contours:
            return contours
        rects = []
        for contour in contours:  # 找出合格的轮廓
            perimeter = cv2.arcLength(contour, True)
            print("S perimeter: ",perimeter)
            if perimeter <= self.contourRange[0] or perimeter >= self.contourRange[1]:  # 对轮廓周长限制
                print("findYellowStick(): to large or to small")
                continue  # 终止本次循环
            w = cv2.boundingRect(contour)[2]
            if 15 >w or w>800:
                print("wide is not qualified",w)
                continue
            rects.append(contour)
        if len(rects) == 0:
            return np.array([])
        contours = rects
        max_contour = contours[0]
        max_perimeter = self.contourRange[0]
        for contour in contours:  # 找出最大的轮廓
            perimeter = cv2.arcLength(contour, True)
            if perimeter > max_perimeter:
                max_perimeter = perimeter
                max_contour = contour
        [x, y, w, h] = cv2.boundingRect(max_contour)
        return [x, y, w, h]

    def get_stick_distance_ancient(self):  # StandInit:49.2, Stand:39.7
        """计算机器人头右转90度时身体到黄杆的距离"""
        camera_direction = -1.2 * rad  # 摄像头和水平线的夹角弧度
        center_y = self.boundRect[1]
        camera_position = self.motionProxy.getPosition("CameraTop", motion.FRAME_ROBOT, True)
        camera_y = camera_position[1]  # 注意右边是负数
        camera_height = camera_position[2] - 0.450  # 减去黄杆的高度,需测量
        head_pitches = self.motionProxy.getAngles("HeadPitch", True)  # 返回传感器角度
        head_pitch = head_pitches[0]
        stick_pitch = (center_y - 240.0) * self.cameraPitchRange / 480.0  # 向下为正
        dis = camera_height / np.tan(camera_direction + head_pitch + stick_pitch)  # SA
        print(dis)
        self.stickDistance = dis - camera_y

    def get_stick_distance(self):
        """用测红球的方法不准，所以根据黄杆的宽度测出机器人到黄杆的距离"""
        camera_position = self.motionProxy.getPosition("CameraTop", motion.FRAME_ROBOT, True)
        camera_y = camera_position[1]  # 注意右边是负数
        half_stick_width = 0.023
        half_angle = self.boundRect[2] * 0.5 * self.cameraYawRange / 640.0  # 向下为正
        dis = half_stick_width / np.tan(half_angle)
        self.stickDistance = dis - camera_y
        print("S ----- dis")

    def get_stick_angle(self, rect):  # 不准但计算速度快 接近正确值
        """黄杆相对于机器人x方向的偏移角度"""
        x = rect[0]
        w = rect[2]
        center_x = x + w * 0.5
        alpha = (320.0 - center_x) / 640.0 * self.cameraYawRange
        head_yaws = self.motionProxy.getAngles("HeadYaw", True)
        head_yaw = head_yaws[0]
        alpha += head_yaw
        self.stickAngle = alpha

    def show_stick(self, save_img=True):
        """显示图片和保存图片"""
        cv2.destroyAllWindows()
        if not self.boundRect:
            print("no stick detected.")
            frame = self.frameArray.copy()
            frame = frame[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪坐标为[y0:y1, x0:x1]
            cv2.imshow("stick position", frame)
            cv2.waitKey(1)
        else:
            [x, y, w, h] = self.boundRect
            frame = self.frameArray.copy()
            frame = frame[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪坐标为[y0:y1, x0:x1]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.circle(frame, (320, 240), 2, (0, 0, 255), 2)  # 画中心点
            cv2.imshow("stick position", frame)
            cv2.waitKey(1)
            if save_img:
                # noinspection PyBroadException
                try:
                    cv2.imwrite("./save_pictures/yellow_stick.jpg", frame)
                except Exception:
                    print("Error when saving current frame!")

class LandMarkDetect(VisualBasis):
    def __init__(self, robot_ip, port=9559, camera_id=vd.kTopCamera):
        super(LandMarkDetect, self).__init__(robot_ip, port, camera_id)
        self.yawAngle = 0
        self.markContour = []  # 用于存储黄杆的矩形信息 [x, y, w, h]
        self.markAngle = 0.0   # 黄杆的角度
        self.cameraID = camera_id
        # 以下新增黄杆检测相关参数
        self.crop = 1  # 图片裁剪比例
        self.contourRange = [100,800]  # 轮廓周长范围
        self.minHsv = np.array([20, 100, 0])  # [20, 100, 40]
        self.maxHsv = np.array([54, 255, 255])  # [44, 255, 255]
        self.cameraProxy.setActiveCamera(self.cameraID)

    def preprocess(self):
        """对当前帧进行预处理(二值化、裁剪等)，以进行黄杆检测"""
        frame_array = self.frameArray
        try:
            frame_array = frame_array[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪
        except IndexError:
            print("error happened when crop the Image!")
        frame_hsv = cv2.cvtColor(frame_array, cv2.COLOR_BGR2HSV)
        frame_bin = cv2.inRange(frame_hsv, self.minHsv, self.maxHsv)
        kernel_erosion = np.ones((5, 5), np.uint8)
        kernel_dilation = np.ones((5, 5), np.uint8)
        frame_bin = cv2.erode(frame_bin, kernel_erosion, iterations=1)  # 腐蚀
        frame_bin = cv2.dilate(frame_bin, kernel_dilation, iterations=1)  # 膨胀
        frame_bin = cv2.GaussianBlur(frame_bin, (3, 3), 0)  # 高斯滤波
        return frame_bin

    def update_mark_data(self):
        """更新黄杆的数据（替代原有的mark数据更新）"""
        self.update_frame()
        frame_bin = self.preprocess()
        contours = self.find_contours(frame_bin)
        contour = self.select_contour(contours)
        if len(contour) == 0:  # 轮廓为空
            self.markContour = []
            self.markAngle = 0.0
        else:
            print("rect:{}".format(contour))
            self.markContour = contour
            self.get_mark_angle(contour)

    @staticmethod
    def find_contours(frame_bin):
        """在预处理的帧中找到黄色的条"""
        contours, _ = cv2.findContours(frame_bin.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours is None:
            return np.array([])
        else:
            return contours

    def select_contour(self, contours):
        """找出最适合的黄杆轮廓"""
        if not contours:
            return contours
        rects = []
        for contour in contours:  # 找出合格的轮廓
            perimeter = cv2.arcLength(contour, True)
            print("M perimeter :",perimeter)
            if perimeter <= self.contourRange[0] or perimeter >= self.contourRange[1]:  # 对轮廓周长限制
                print("findYellowStick(): to large or to small")
                continue  # 终止本次循环
            w = cv2.boundingRect(contour)[2]
            if 15 >w or w>800:  # 对宽进行限制
                print("wide is not qualified",w)
                continue
            rects.append(contour)
        if len(rects) == 0:
            return np.array([])
        contours = rects
        max_contour = contours[0]
        max_perimeter = self.contourRange[0]
        for contour in contours:  # 找出最大的轮廓
            perimeter = cv2.arcLength(contour, True)
            if perimeter > max_perimeter:
                max_perimeter = perimeter
                max_contour = contour
        [x, y, w, h] = cv2.boundingRect(max_contour)
        return [x, y, w, h]

    def get_mark_angle(self, rect):
        """通过黄杆轮廓找出mark偏移的弧度"""
        x = rect[0]
        w = rect[2]
        center_x = x + w * 0.5
        mark_yaw = (320.0 - center_x) * self.cameraYawRange / 640.0  # 向左为正
        head_yaws = self.motionProxy.getAngles("HeadYaw", True)
        head_yaw = head_yaws[0]
        mark_yaw += head_yaw
        self.markAngle = mark_yaw

    def show_mark(self, save_img=True):
        """显示黄杆检测结果"""
        cv2.destroyAllWindows()
        if not self.markContour:
            print("no yellow stick detected.")
            frame = self.frameArray.copy()
            frame = frame[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪
            cv2.imshow("yellow stick position", frame)
            cv2.waitKey(1)
        else:
            [x, y, w, h] = self.markContour
            frame = self.frameArray.copy()
            frame = frame[int((1 - self.crop) * self.frameHeight):, :]  # 裁剪
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # 标记中心点
            center_x = x + w * 0.5
            center_y = y + h * 0.5
            cv2.circle(frame, (int(center_x), int(center_y)), 2, (0, 0, 255), 2)
            cv2.imshow("yellow stick position", frame)
            cv2.waitKey(1)
            if save_img:
                # noinspection PyBroadException
                try:
                    cv2.imwrite("./save_pictures/mark.jpg", frame)
                except Exception:
                    print("Error when saving current frame!")

# class LandMarkDetect(VisualBasis):
#     def __init__(self, robot_ip, port=9559, camera_id=vd.kTopCamera):
#         super(LandMarkDetect, self).__init__(robot_ip, port, camera_id)
#         self.yawAngle = 0
#         self.markContour = []
#         self.markAngle = 0.0
#         self.url = "http://127.0.0.1:5000/index"
#
#     def update_mark_data(self):
#         """更新mark的数据"""
#         self.update_frame()
#         cv2.imwrite("./save_pictures/mark.jpg", self.frameArray)  # 保存所拍摄的mark图片用来识别
#         resp = urllib.urlopen(self.url)  # 获取yolov5检测的数据
#         string = resp.readlines()[0]
#         data = json.loads(string)
#         self.markContour = []  # 每次刷新数组
#         if data:  # 找到mark
#             self.markContour.append(data['pt1'][0])
#             self.markContour.append(data['pt1'][1])
#             self.markContour.append(data['pt2'][0] - self.markContour[0])
#             self.markContour.append(data['pt2'][1] - self.markContour[1])
#             self.get_mark_angle()
#         else:
#             print("no mark detected.")
#             self.markContour = []
#             self.markAngle = 0.0
#
#     def get_mark_angle(self):
#         """通过图像中的红色方框轮廓找出mark偏移的弧度"""
#         x = self.markContour[0]
#         w = self.markContour[2]
#         center_x = x + w * 0.5
#         mark_yaw = (320.0 - center_x) * self.cameraYawRange / 640.0  # 向左为正
#         head_yaws = self.motionProxy.getAngles("HeadYaw", True)
#         head_yaw = head_yaws[0]
#         mark_yaw += head_yaw
#         self.markAngle = mark_yaw
#
#     @staticmethod
#     def show_mark():  # 显示yolov5识别的mark
#         cv2.destroyAllWindows()
#         img_source = cv2.imread("./save_pictures/yolov5.jpg")
#         cv2.imshow("yolov5_mark", img_source)
#         cv2.waitKey(1)


class WhiteLine(VisualBasis):
    def __init__(self, robot_ip, port=9559, camera_id=vd.kBottomCamera):
        super(WhiteLine, self).__init__(robot_ip, port, camera_id)
        self.lineRect = []
        self.lineRadian = 0.0  # 弧度
        self.w_crop = 0.75
        self.h_crop = 0.85

    def update_line_data(self):
        """从指定摄像机更新白线数据"""
        self.update_frame()
        frame_array = self.frameArray
        frame = frame_array[int((1 - self.h_crop) * self.frameHeight):, int((1 - self.w_crop) * self.frameWidth):]  # 裁剪
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.GaussianBlur(gray, (3, 3), 0)
        kernel = np.ones((3, 3), np.uint8)
        frame = cv2.erode(frame, kernel, iterations=1)  # 腐蚀
        frame = cv2.dilate(frame, kernel, iterations=3)  # 膨胀
        edges = cv2.Canny(frame, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 60)
        if lines is None:
            self.lineRect = []
            self.lineRadian = 0.0
        else:
            self.lineRect = lines[0][0]  # 默认第一个值
            self.lineRadian = 90 * rad - lines[0][0][1]

    def show_line(self, save_img=True):
        cv2.destroyAllWindows()
        frame_array = self.frameArray
        frame = frame_array[int((1 - self.h_crop) * self.frameHeight):, int((1 - self.w_crop) * self.frameWidth):]  # 裁剪
        if len(self.lineRect) == 0:
            cv2.imshow("line", frame)
            cv2.waitKey(1)
        else:
            rho = self.lineRect[0]  # 像素长度
            theta = self.lineRect[1]  # 弧度
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
        if save_img:
            # noinspection PyBroadException
            try:
                cv2.imwrite("./save_pictures/white_line.jpg", frame)
            except Exception:
                print("Error when saving current frame!")


"""下面为测试用的代码"""
# from naoqi import ALProxy
# IP = "192.168.0.114"
# motionProxy = ALProxy("ALMotion", IP, 9559)
# motionProxy.wakeUp()
# motionProxy.moveInit()
# ball = BallDetect(IP)
# stick = StickDetect(IP)
# line = WhiteLine(IP)
# mark = LandMarkDetect(IP)
# motionProxy.angleInterpolationWithSpeed("HeadYaw", 0 * rad, 0.6)
# motionProxy.angleInterpolationWithSpeed("HeadPitch", 0 * rad, 0.6)
# # time.sleep(5)
# while True:
#     for i in range(5):
#         mark.update_mark_data()
#         mark.show_mark()
#         print(mark.markAngle / rad)
#         motionProxy.moveTo(0, 0, mark.markAngle)
#         mark.delete_mark_img()
#     break
# cv2.destroyAllWindows()
# motionProxy.rest()

# url = "http://127.0.0.1:5000/index"
# resp = urllib.urlopen(url)
# string = resp.readlines()[0]
# data = json.loads(string)
# if data:
#     print("hello world")
# else:
#     print("none")
