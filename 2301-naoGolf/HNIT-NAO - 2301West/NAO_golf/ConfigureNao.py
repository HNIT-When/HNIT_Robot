# coding=utf-8
"""
NAO的配置文件，基类，初始化代理
"""
import sys  # 这个模块可供访问由解释器使用或维护的变量和与解释器进行交互的函数
import os  # 这个模块提供了一种方便的使用操作系统函数的方法
import codecs  # 主要用于在不同数据之间转换文本的编码器和解码器
from naoqi import ALProxy


class ConfigureNao(object):
    def __init__(self, robot_ip, port=9559):
        self.IP = robot_ip
        self.PORT = port
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
