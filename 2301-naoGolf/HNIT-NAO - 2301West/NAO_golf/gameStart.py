# coding=utf-8
import numpy as np
import almath
import threading  # 线程模块
import ctypes  # 提供和C语言兼容的数据类型
import inspect  # 用于收集 python 对象的信息，可以获取类或 函数 的参数的信息，源码，解析堆栈，对对象进行类型检查等等
import vision_definitions as vd
import time

from ConfigureNao import ConfigureNao
from visualTask import BallDetect
from visualTask import StickDetect
from visualTask import LandMarkDetect
from visualTask import WhiteLine

"""运动附加函数"""
from step import walk_step,walk_step2
from batting import Batting

rad = almath.TO_RAD  # 1度所对应的弧度数 0.0174532923847


def async_raise(tid, exit_type):
    """引发异常，根据需要执行清理"""
    try:
        tid = ctypes.c_long(tid)  # C语言的longlong类型
        if not inspect.isclass(exit_type):  # 判断一个对象是否是类,如果object是一个class就返回True，反之则返回False
            exit_type = type(exit_type)  # type() 函数返回指定对象的类型
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exit_type))
        if res == 0:
            pass
            # raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble"""
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)  # 强制杀掉线程
            # raise SystemError("PyThreadState_SetAsyncExc failed")
    except Exception as err:
        print(err)


def stop_thread(thread):
    """终止线程"""
    async_raise(thread.ident, SystemExit)


class GolfGame(ConfigureNao):
    def __init__(self, robot_ip, port=9559):
        super(GolfGame, self).__init__(robot_ip, port)  # 子类赋值给父类
        self.IP = robot_ip
        self.PORT = port
        self.ball_info = [0, 0, 0]  # 球的信息,[disX, disY, angle]
        self.stickAngle = 0.0  # 杆的方位角信息
        self.markAngle = 0.0  # mark的方位角信息
        self.pitchAngle = 0  # 机器人头的俯角
        self.yawAngle = 0  # 机器人头的方位角

        self.ballDetect = BallDetect(robot_ip, camera_id=vd.kBottomCamera)  # 红球检测
        self.stickDetect = StickDetect(robot_ip, camera_id=vd.kTopCamera)  # 黄杆检测
        self.landMarkDetect = LandMarkDetect(robot_ip, camera_id=vd.kTopCamera)  # 地标检测
        self.whiteLine = WhiteLine(robot_ip, camera_id=vd.kBottomCamera)  # 白线检测
        self.batting = Batting(robot_ip)

        self.compensateAngleRat_l = 0.08  # 最后一球的时候补偿角 左偏为正值 右偏为负值
        self.compensateAngleRat_r = 0.08  # 最后一球的时候补偿角
        self.task1_hitBallSpeed1 =2.00  # 第一关第一杆击球速度
        self.task2_hitBallSpeed1 = 1.57 # 第二关第一杆的击球速度 1.80 -> 1.60
        self.task2_hitBallSpeed2 = 2.70  # 第二关第二杆击球速度
        self.task3_hitBallSpeed1 = 3.50  # 第三关第一杆的击球速度
        self.task3_hitBallSpeed2 = 2.60  # 第三关第二杆的击球速度 1.85
        self.task3_hitBallSpeed3 = 2.70  # 第三关第三杆的击球速度

        self.t1 = None  # 三个任务都开启一个线程，实际并不会同时运行，只是方便需要强制停止当前任务时，强制终止线程
        self.t2 = None
        self.t3 = None

        self.prepare()
    def prepare(self):
        """开始准备"""
        self.motionProxy.wakeUp()
        self.motionProxy.openHand("RHand")
        self.postureProxy.goToPosture("StandInit", 0.6)
        self.motionProxy.setStiffnesses("Body", 1.0)
        self.motionProxy.setMoveArmsEnabled(False, False)  # 手臂不动
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)
        self.tts.say("请给我球杆")
        self.batting.ready_for_catch()
        while True:
            front_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
            middle_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
            rear_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
            if front_flag or middle_flag or rear_flag:
                break
        self.batting.catch_golf_club()
        self.start()
    def start(self):
        """开始"""
        t1_alive = False
        t2_alive = False
        t3_alive = False
        self.tts.say("请选择关卡")
        while True:
            front_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
            middle_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
            rear_flag = self.memoryProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
            # left_foot_flag = self.memoryProxy.getData("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value")  # 左脚左传感器
            # left_hand_flag = self.memoryProxy.getData("Device/SubDeviceList/LHand/Touch/Back/Sensor/Value")  # 有问题
            # right_hand_flag = self.memoryProxy.getData("Device/SubDeviceList/RHand/Touch/Right/Sensor/Value")
            if front_flag and t1_alive:
                self.tts.say("第一关结束")
                print("task1 is terminate\n")
                stop_thread(self.t1)
                time.sleep(2)
                self.prepare()
                t1_alive = False

            if middle_flag and t2_alive:
                self.tts.say("第二关结束")
                print("task2 is terminate\n")
                stop_thread(self.t2)
                time.sleep(2)
                self.prepare()
                t2_alive = False

            if rear_flag and t3_alive:
                self.tts.say("第三关结束")
                print("task3 is terminate\n")
                stop_thread(self.t3)
                time.sleep(2)
                self.prepare()
                t3_alive = False

            if front_flag and not t1_alive and not t2_alive and not t3_alive:
                self.tts.say("第一关开始")
                print("task1 stat")
                self.t1 = threading.Thread(target=self.task1_start)
                self.t1.start()  # 启动1线程
                time.sleep(2)
                t1_alive = True

            if middle_flag and not t1_alive and not t2_alive and not t3_alive:
                self.tts.say("第二关开始")
                print("task2 stat")
                self.t2 = threading.Thread(target=self.task2_start)
                self.t2.start()  # 启动2线程
                time.sleep(2)
                t2_alive = True

            if rear_flag and not t1_alive and not t2_alive and not t3_alive:
                self.tts.say("第三关开始")
                print("task3 stat")
                self.t3 = threading.Thread(target=self.task3_start)
                self.t3.start()  # 启动3线程
                time.sleep(2)
                t3_alive = True
#新机器人
    def task1_start(self):
        """第一关"""
        self.motionProxy.moveTo(1.0, 0, 0,walk_step())
        self.walk_to_ball_TEST2()
        #self.mark_ball_robot()
        self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, 40 * rad, walk_step())
        self.motionProxy.moveTo(-0.18, 0, 0, walk_step())  # 后退
        self.motionProxy.moveTo(0, -0.2, 0, walk_step())  # 斜着走
        self.line_ball_robot_TST1(0)  # 向右偏
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task1_hitBallSpeed1)
        self.batting.finish_hitting()  # 理论上一球进洞

    def task2_start(self):
        """第二关"""
        #self.motionProxy.moveTo(0,-0.55, 0, walk_step())
        self.motionProxy.moveTo(0.55,0, 0, walk_step())
        self.motionProxy.moveTo(0, 0, 85 * rad, walk_step())

        # self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())
        # self.motionProxy.moveTo(0, -0.35, 0, walk_step())
        # self.motionProxy.moveTo(0, 0, 50 * rad, walk_step())
        # self.motionProxy.moveTo(0, -0.30, 0, walk_step())  # 斜着走
        self.line_ball_robot(0.00)  # 增加调节参数
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task2_hitBallSpeed1)  # 第一球
        self.batting.finish_hitting()
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())  # 向右转90度


        self.motionProxy.moveTo(0.7, 0, 0, walk_step())

        #self.walk_to_stick(1)
        self.mark_ball_robot_TEST2()
        self.motionProxy.moveTo(0.0, -0.55, 0, walk_step())
        self.motionProxy.moveTo(0.30, 0, 0, walk_step())
        self.mark_ball_robot_TEST2()
        self.motionProxy.moveTo(0.60, 0, 0, walk_step2())
        self.mark_ball_robot_TEST2()

        #self.walk_to_stick()
        #self.tts.say("已靠经黄杆")
        self.walk_to_ball()
        self.move_head_to_find_mark([20],yaw_angles=[-45, -15, 15, 45, -90, 90])  # 头先右边再左边
        # if self.markAngle > 0.5:  # 杆在左边
        #     print("right,markAngle:{}".format(self.markAngle))
        #     self.adjust_to_ball_right_TEST2()
        # else:
        #     print("left,markAngle:{}".format(self.markAngle))
        self.adjust_to_ball_left_TEST2()
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task3_hitBallSpeed3)  # 第二球
        self.batting.finish_hitting()

    def task3_start(self):
        """第三关"""
        self.motionProxy.moveTo(0.55, 0, 0, walk_step())
        self.motionProxy.moveTo(0, 0, 85 * rad, walk_step())
        self.line_ball_robot(deviation=-0.10)  # 右斜对着白线
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task3_hitBallSpeed1)  # 第一球
        self.batting.finish_hitting()
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())
        self.motionProxy.moveTo(1, 0, 0, walk_step())
        self.walk_to_ball()
        self.triangle_location(yaw_angles=[-115,-90,-75])  # 三角定位
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task3_hitBallSpeed2)  # 第二球
        self.batting.finish_hitting()
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, -45 * rad, walk_step())  # 向右转90度

        self.motionProxy.moveTo(1.5, 0, 0, walk_step())
        self.mark_ball_robot_TEST2()
        self.walk_to_ball()
        self.move_head_to_find_mark([20], yaw_angles=[-45, -15, 15, 45, -90, 90])  # 头先右边再左边
        # self.move_head_to_find_mark(yaw_angles=[-45, -15, 15, 45, -90, 90])  # 头先右边再左边
        # if self.markAngle > 0.5:  # 杆在左边
        #     print("right,markAngle:{}".format(self.markAngle))
        #     self.adjust_to_ball_right()
        # else:
        #     print("left,markAngle:{}".format(self.markAngle))
        #     self.adjust_to_ball_left_TEST2()
        self.adjust_to_ball_left_TEST2()
        self.tts.say("开始击球")
        self.batting.ready_for_hit()
        self.batting.hit_ball(self.task3_hitBallSpeed3)  # 第三球
        self.batting.finish_hitting()

    """public function"""

    def walk_to_ball(self):
        """先找球再往球的方向走"""
        if self.find_ball() is False:
            while True:
                is_find_ball = self.move_head_to_find_ball()
                if is_find_ball:
                    break
                else:
                    self.tts.say("没找到红球")
                    self.motionProxy.moveTo(0.50, 0, 0, walk_step())  # 直走
        self.tts.say("找到红球")
        while True:
            if self.ball_info[0] > 0.50:
                self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
                self.motionProxy.moveTo(0.30, 0, 0, walk_step())
            elif self.ball_info[0] > 0.30:
                self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
                self.motionProxy.moveTo(0.15, 0, 0, walk_step())
            else:
                break
            self.find_ball()
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
        self.motionProxy.moveTo(self.ball_info[0] - 0.20, 0, 0, walk_step())  # 走到距离球20零米的位置
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0 * rad, 0.6)
        self.tts.say("已靠经红球")
    def walk_to_ball_TEST2(self):
        """先找球再往球的方向走"""
        if self.find_ball() is False:
            while True:
                is_find_ball = self.move_head_to_find_ball()
                if is_find_ball:
                    break
                else:
                    self.tts.say("没找到红球")
                    self.motionProxy.moveTo(0.50, 0, 0, walk_step())  # 直走
        self.tts.say("找到红球")
        while True:
            if self.ball_info[0] > 0.50:
                self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
                self.motionProxy.moveTo(0.30, 0, 0, walk_step())
            elif self.ball_info[0] > 0.30:
                self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
                self.motionProxy.moveTo(0.15, 0, 0, walk_step())
            else:
                break
            self.find_ball()
        # self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        # self.find_ball()
        # self.motionProxy.moveTo(0, 0, self.ball_info[2], walk_step())
        # self.motionProxy.moveTo(self.ball_info[0] - 0.20, 0, 0, walk_step())  # 走到距离球20零米的位置
        # self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0 * rad, 0.6)
        # self.tts.say("已靠经红球")

    def walk_to_stick(self, t=3):
        """朝黄杆走"""
        if not self.find_mark():
            self.move_head_to_find_mark(yaw_angles=[40, 15, -15, -40])
        self.motionProxy.moveTo(0, 0, self.markAngle, walk_step())
        for i in range(t):  # 走0.3m停一下再走0.3m循环t次
            self.motionProxy.moveTo(0.50, 0, 0, walk_step())
            self.find_mark()
            self.motionProxy.moveTo(0, 0, self.markAngle, walk_step())  # 身体对准黄杆

    def find_ball(self, is_show_flag=True):
        """找红球"""
        self.ballDetect.update_ball_data()
        self.ball_info = self.ballDetect.get_ball_position()
        if is_show_flag is True:
            self.ballDetect.show_ball_position()
        if self.ball_info[0] > 0:
            return True
        else:
            return False

    def find_stick(self, is_show_flag=True):
        """找黄杆"""
        self.stickDetect.update_stick_data()
        self.stickAngle = self.stickDetect.stickAngle
        if is_show_flag is True:
            self.stickDetect.show_stick()
        if self.stickDetect.boundRect:
            return True
        else:
            return False

    def find_mark(self, is_show_mark=True):
        """找mark"""
        self.landMarkDetect.update_mark_data()
        self.markAngle = self.landMarkDetect.markAngle
        if is_show_mark is True:
            self.landMarkDetect.show_mark()
        if self.landMarkDetect.markContour:
            return True
        else:
            return False

    def move_head_to_find_ball(self, pitch_angles=None, yaw_angles=None):
        """转头找球"""
        names = ["HeadPitch", "HeadYaw"]
        if pitch_angles is None:
            pitch_angles = [0]
        if yaw_angles is None:
            yaw_angles = [40, 15, -15, -40]
        is_find_ball = False
        for pitch_angle in pitch_angles:
            for yaw_angle in yaw_angles:
                self.pitchAngle = pitch_angle
                self.yawAngle = yaw_angle
                angles = [pitch_angle * rad, yaw_angle * rad]  # 弧度
                self.motionProxy.angleInterpolationWithSpeed(names, angles, 0.4)
                is_find_ball = self.find_ball()
                if is_find_ball:
                    break
            if is_find_ball:
                break
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)  # 转回来
        return is_find_ball

    def move_head_to_find_mark(self, pitch_angles=None, yaw_angles=None, fixed=False):
        """转头找杆并对准mark的方向"""
        names = ["HeadPitch", "HeadYaw"]
        if pitch_angles is None:
            pitch_angles = [0]
        if yaw_angles is None:
            yaw_angles = [-15, +15]
        is_find_mark = False
        for pitch_angle in pitch_angles:
            for yaw_angle in yaw_angles:
                self.pitchAngle = pitch_angle
                self.yawAngle = yaw_angle
                angles = [pitch_angle * rad, yaw_angle * rad]  # 弧度
                self.motionProxy.angleInterpolationWithSpeed(names, angles, 0.6)
                is_find_mark = self.find_mark()
                if is_find_mark:
                    break
            if is_find_mark:
                break
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", self.markAngle, 0.6)  # 头转到杆的方向
        if not fixed:  # 是否固定头部
            self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)  # 转回来
        return is_find_mark

    def triangle_location(self, yaw_angles):
        """三角定位"""
        self.move_head_to_find_mark(yaw_angles=yaw_angles)
        print("angle_x:{}".format(self.markAngle / rad))
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)  # 转回来
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())  # 侧面对准mark
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 距红球0.15
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 使红球位于右脚前面
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
    def mark_ball_robot_TEST2(self):
        """调整位置使身体正对黄杆和红球的直线"""
        if not self.find_mark():  # 黄杆加mark
            print("未找到mark")
            self.move_head_to_find_mark(yaw_angles=[15, -15])
        print("angel_x:{}".format(self.markAngle / rad))
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)  # 转回来
        self.motionProxy.moveTo(0, 0, self.markAngle, walk_step())  # 对准mark

    def mark_ball_robot(self):
        """调整位置使身体正对黄杆和红球的直线"""
        if not self.find_mark():  # 黄杆加mark
            print("未找到mark")
            self.move_head_to_find_mark(yaw_angles=[15, -15])
        print("angel_x:{}".format(self.markAngle / rad))
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.6)  # 转回来
        self.motionProxy.moveTo(0, 0, self.markAngle, walk_step())  # 对准mark
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(0, self.ball_info[1], 0, walk_step())  # 斜着走
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0 * rad, 0.6)

    def line_ball_robot(self, deviation=0.0):  # 想要机器人向右传负值，左边传正值
        """机器人平行白线并对准红球"""
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())#后退
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        for i in range(4):  # 根据白线转到指定角度, 最大循环5次
            self.whiteLine.update_line_data()
            self.whiteLine.show_line()
            self.whiteLine.lineRadian += deviation  # 第三关向右偏移,弧度
            print("LineRadian:{}, LineAngle:{}".format(self.whiteLine.lineRadian, self.whiteLine.lineRadian / rad))
            if abs(self.whiteLine.lineRadian) < 0.015 and len(self.whiteLine.lineRect):
                break
            self.motionProxy.moveTo(0, 0, self.whiteLine.lineRadian, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 距红球0.15
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0,walk_step())  # 使红球位于右脚前面
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
    def line_ball_robot_TST1(self, deviation=0.0):  # 想要机器人向右传负值，左边传正值
        """机器人平行白线并对准红球"""
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())#后退
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        for i in range(5):  # 根据白线转到指定角度, 最大循环5次
            self.whiteLine.update_line_data()
            self.whiteLine.show_line()
            self.whiteLine.lineRadian += deviation  # 第三关向右偏移,弧度
            print("LineRadian:{}, LineAngle:{}".format(self.whiteLine.lineRadian, self.whiteLine.lineRadian / rad))
            if abs(self.whiteLine.lineRadian) < 0.015 and len(self.whiteLine.lineRect):
                break
            self.motionProxy.moveTo(0, 0, self.whiteLine.lineRadian, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 距红球0.15
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0,walk_step())  # 使红球位于右脚前面
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)

    def adjust_to_ball_left(self):
        """球在左边转到合适的击球位置"""
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第一次调整
        while not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.10, 0, 0, walk_step())
            self.motionProxy.moveTo(0, -0.03, 0, walk_step())
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第二次调整
        while not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15], fixed=True)  # 找到mark后固定
        self.find_stick()
        self.stickDetect.get_stick_distance()  # 黄杆测距
        angle = np.arctan2(self.ball_info[0], self.stickDetect.stickDistance)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0 * rad, 0.6)
        self.motionProxy.moveTo(0, 0, -angle + self.compensateAngleRat_l, walk_step())  # 增加补偿角
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第三次调整
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)

    def adjust_to_ball_right(self):
        """球在右边转到合适的击球位置"""
        self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())  # 向左转90度
        self.motionProxy.moveTo(-0.25, 0, 0, walk_step())  # 后退
        self.motionProxy.moveTo(0, -0.20, 0, walk_step())  # 斜着走
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.1, 0, 0, walk_step())
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.20, 0, 0, walk_step())
        self.motionProxy.moveTo(0, self.ball_info[1], 0, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-45, -15, 15, 45, -90, 90])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.moveTo(-0.20, 0, 0, walk_step())  # 后退
        self.motionProxy.moveTo(0, -0.15, 0, walk_step())  # 斜着走
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15], fixed=True)  # 找到mark后固定
        self.find_stick()
        self.stickDetect.get_stick_distance()  # 黄杆测距
        angle = np.arctan2(self.ball_info[0], self.stickDetect.stickDistance)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0 * rad, 0.6)
        self.motionProxy.moveTo(0, 0, -angle + self.compensateAngleRat_l, walk_step())  # 增减补偿角
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)

    def adjust_to_ball_left_TEST2(self):
        """球在左边转到合适的击球位置"""
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第一次调整
        while not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.10, 0, 0, walk_step())
            self.motionProxy.moveTo(0, -0.10, 0, walk_step())
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark([20],yaw_angles=[-15, -45, -90])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第二次调整
        while not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark([30],yaw_angles=[ -15, -90], fixed=True)  # 找到mark后固定
        self.find_stick()
        self.stickDetect.get_stick_distance()  # 黄杆测距
        angle = np.arctan2(self.ball_info[0], self.stickDetect.stickDistance)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0 * rad, 0.6)
        self.motionProxy.moveTo(0, 0, -angle + self.compensateAngleRat_l, walk_step())  # 增加补偿角
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头第三次调整
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
    def adjust_to_ball_right_TEST2(self):
        """球在右边转到合适的击球位置"""
        self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())
        self.motionProxy.moveTo(0, 0, 45 * rad, walk_step())  # 向左转90度
        self.motionProxy.moveTo(-0.25, 0, 0, walk_step())  # 后退
        self.motionProxy.moveTo(0, -0.20, 0, walk_step())  # 斜着走
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.1, 0, 0, walk_step())
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.20, 0, 0, walk_step())
        self.motionProxy.moveTo(0, self.ball_info[1], 0, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-45,-90])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.moveTo(-0.20, 0, 0, walk_step())  # 后退
        self.motionProxy.moveTo(0, -0.15, 0, walk_step())  # 斜着走
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        if not self.find_ball():  # 如果没看到红球
            self.motionProxy.moveTo(-0.05, 0, 0, walk_step())
            self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15])
        self.motionProxy.moveTo(0, 0, self.markAngle + 90 * rad, walk_step())
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
        self.move_head_to_find_mark(yaw_angles=[-115, -75, -45, -15], fixed=True)  # 找到mark后固定
        self.find_stick()
        self.stickDetect.get_stick_distance()  # 黄杆测距
        angle = np.arctan2(self.ball_info[0], self.stickDetect.stickDistance)
        self.motionProxy.angleInterpolationWithSpeed("HeadYaw", 0 * rad, 0.6)
        self.motionProxy.moveTo(0, 0, -angle + self.compensateAngleRat_l, walk_step())  # 增减补偿角
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 25 * rad, 0.6)  # 低头
        self.find_ball()
        self.motionProxy.moveTo(self.ball_info[0] - 0.15, 0, 0, walk_step())  # 调整x
        self.motionProxy.moveTo(0, self.ball_info[1] + 0.06, 0, walk_step())  # 调整y
        self.motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.6)
if __name__ == "__main__":
    time.sleep(1)
    golf = GolfGame('192.168.206.175')
