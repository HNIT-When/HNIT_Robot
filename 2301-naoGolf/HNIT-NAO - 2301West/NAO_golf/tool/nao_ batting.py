# coding=utf-8
from ConfigureNao import ConfigureNao
import time
from batting import Batting


class NaoBatting(ConfigureNao):
    def __init__(self, robot_ip, port=9559):
        super(NaoBatting, self).__init__(robot_ip, port)  # 子类赋值给父类
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


IP = "192.168.89.26"
if __name__ == "__main__":
    hit_ball = NaoBatting(IP)
    # hit_ball.golf_club()
    while True:
        t = raw_input("(退出输入q,接杆输入g)\n请输入击球的时间t:")
        if t == "q":
            hit_ball.batting.ready_for_catch()
            break
        if t == "g":
            hit_ball.golf_club()
            continue
        try:
            t = float(t)
        except ValueError:
            print("\n请输入数字或设定的命令!\n")
            continue
        hit_ball.tts.say("开始击球")
        hit_ball.batting.ready_for_hit()
        hit_ball.batting.hit_ball(t)
        hit_ball.batting.finish_hitting()
    hit_ball.motionProxy.rest()
