# coding=utf-8
from naoqi import ALProxy
import time
import numpy as np
import almath

IP = "192.168.216.26"  # 机器人的IP地址
PORT = 9559  # 机器人的端口号，默认9559
rad = almath.TO_RAD  # 1度所对应的弧度数 57.2957801819

# def walk_slow():
#     walk_slow = [["MaxStepFrequency", 0.98],
#                  ["MaxStepX", 0.035],
#                  ["MaxStepY", 0.11],
#                  ["MaxStepTheta", 0.22],
#                  ["StepHeight", 0.0135],
#                  ["TorsoWx", 0.00],
#                  ["TorsoWy", 0.00]
#                  ]
#     return walk_slow

motion = ALProxy("ALMotion", IP, PORT)
postureProxy = ALProxy("ALRobotPosture", IP, PORT)  # 预定义姿势

# time.sleep(2)
motion.wakeUp()

# motion.moveInit()
postureProxy.goToPosture("StandInit", 0.5)
# motion.setStiffnesses("Body", 1.0)
# motion.setMoveArmsEnabled(False, False)  # 手臂不动

# motion.moveTo(0, 0, 45 * rad, walk_slow())  # 往前
# motion.setAngles("HeadPitch", 80/180*np.pi, 0.5)
# head_pitches = motion.getAngles("HeadPitch", True)
# head_pitch = head_pitches[0]
# print(head_pitch/np.pi*180)

time.sleep(2)
motion.rest()
