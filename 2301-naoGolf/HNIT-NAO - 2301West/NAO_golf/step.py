# coding=utf-8
"""
步态参数配置
"""
from ConfigureNao import ConfigureNao
import time
from batting import Batting

####
import almath
import socket
import shutil
# from step import walk_step

global channelB, channelG, channelR
rad = almath.TO_RAD  # 1度所对应的弧度数 0.0174532923847


class WalkConfiguration(ConfigureNao):
    def __init__(self, robot_ip, port=9559):
        super(WalkConfiguration, self).__init__(robot_ip, port)  # 子类赋值给父类
        self.batting = Batting(robot_ip)

    def golf_club(self):
        self.motionProxy.wakeUp()
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


# 没贴标签的机器人###################################################################################
# def walk_step():
#     walk_step_ = [["MaxStepFrequency", 0.80],
#                   ["MaxStepX", 0.025],
#                   ["MaxStepY", 0.12],
#                   ["MaxStepTheta", 0.22],
#                   ["StepHeight", 0.0135],
#                   ["TorsoWx", 0.00],
#                   ["TorsoWy", 0.00]]
#     return walk_step_
def walk_step():
    walk_step_ = [["MaxStepFrequency", 0.90],
                  ["MaxStepX", 0.030],
                  ["MaxStepY", 0.12],
                  ["MaxStepTheta", 0.2],
                  ["StepHeight", 0.015],
                  ["TorsoWx", 0.00],
                  ["TorsoWy", 0.00]]
    return walk_step_
def walk_step2():
    walk_step_ = [["MaxStepFrequency", 0.40],
                  ["MaxStepX", 0.030],
                  ["MaxStepY", 0.12],
                  ["MaxStepTheta", 0.2],
                  ["StepHeight", 0.012],
                  ["TorsoWx", 0.00],
                  ["TorsoWy", 0.05]]
    return walk_step_

# 贴了标签的机器人###################################################################################
# def walk_step():
#     walk_step_ = [["MaxStepFrequency", 0.82],
#                   ["MaxStepX", 0.025],
#                   ["MaxStepY", 0.12],
#                   ["MaxStepTheta", 0.00],  # 0.22
#                   ["StepHeight", 0.013],
#                   ["TorsoWx", 0.00],
#                   ["TorsoWy", 0.00]]
#     return walk_step_


IP = "192.168.11.13"
if __name__ == "__main__":
    walk = WalkConfiguration(IP)

    walk.golf_club()  # 接杆

    walk.motionProxy.moveTo(1, 0, 0, walk_step())#第一个为距离，第二个为

    # walk.batting.ready_for_catch()

    # walk.batting.ready_for_hit()
    # walk.batting.hit_ball(4)
    # walk.batting.finish_hitting()

    walk.motionProxy.rest()
