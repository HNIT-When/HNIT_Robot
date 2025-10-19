# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from naoqi import ALProxy


class NAOMarkDetector:
    def __init__(self, robot_ip, port=9559):
        """
        初始化NAO Mark检测器
        :param robot_ip: NAO机器人的IP地址
        :param port: 端口号，默认9559
        """
        self.robot_ip = robot_ip
        self.port = port

        # 创建NAOqi代理
        try:
            self.memory_proxy = ALProxy("ALMemory", robot_ip, port)
            self.landmark_proxy = ALProxy("ALLandMarkDetection", robot_ip, port)
            self.video_proxy = ALProxy("ALVideoDevice", robot_ip, port)
            print "成功连接到NAO机器人: %s" % robot_ip
        except Exception as e:
            print "连接NAO机器人失败: %s" % e
            return

        # 检测状态
        self.is_detecting = False
        self.camera_handle = None

    def start_detection(self):
        """开始Mark检测"""
        try:
            self.landmark_proxy.subscribe("MarkDetector", 500, 0.0)
            self.is_detecting = True
            print "Mark检测已启动"
        except Exception as e:
            print "启动Mark检测失败: %s" % e

    def stop_detection(self):
        """停止Mark检测"""
        try:
            if self.is_detecting:
                self.landmark_proxy.unsubscribe("MarkDetector")
                self.is_detecting = False
                print "Mark检测已停止"
        except:
            self.is_detecting = False

    def get_mark_data(self):
        """获取Mark数据"""
        try:
            return self.memory_proxy.getData("LandmarkDetected")
        except:
            return None

    def setup_camera(self, camera_id=0):
        """设置摄像头"""
        try:
            self.camera_handle = self.video_proxy.subscribeCamera(
                "Camera_Client", camera_id, 2, 11, 15)
            return True
        except Exception as e:
            print "摄像头设置失败: %s" % e
            return False

    def get_camera_image(self):
        """获取摄像头图像"""
        try:
            if self.camera_handle is None:
                return None

            image_data = self.video_proxy.getImageRemote(self.camera_handle)
            if image_data is None:
                return None

            width = image_data[0]
            height = image_data[1]
            image_array = image_data[6]

            # 转换为numpy数组
            image_np = np.frombuffer(image_array, dtype=np.uint8)
            image_np = image_np.reshape((height, width, 3))
            image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

            return image_np
        except:
            return None

    def cleanup_camera(self):
        """清理摄像头资源"""
        try:
            if self.camera_handle is not None:
                self.video_proxy.unsubscribe(self.camera_handle)
        except:
            pass

    def parse_mark_data_simple(self, mark_data):
        """简化版Mark数据解析"""
        marks = []

        # 检查数据有效性
        if (mark_data and isinstance(mark_data, list) and
                len(mark_data) >= 2 and isinstance(mark_data[1], list)):

            for mark in mark_data[1]:
                try:
                    # 基本数据提取
                    if len(mark) >= 3:
                        mark_id = mark[0]
                        position = mark[2]  # [x, y, z]

                        if len(position) >= 3:
                            marks.append({
                                'id': mark_id,
                                'x': position[0],
                                'y': position[1],
                                'z': position[2],
                                'distance': np.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)
                            })
                except:
                    continue

        return marks

    def draw_detection_info(self, image, marks):
        """在图像上绘制检测信息"""
        if image is None:
            return image

        display_image = image.copy()
        height, width = display_image.shape[:2]

        # 绘制中心十字线
        cv2.line(display_image, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)
        cv2.line(display_image, (0, height // 2), (width, height // 2), (0, 255, 0), 1)

        # 绘制检测到的Mark
        for mark in marks:
            try:
                # 简化的3D到2D投影
                x_3d, y_3d, z_3d = mark['x'], mark['y'], mark['z']

                if z_3d > 0:
                    focal = 500
                    center_x, center_y = width // 2, height // 2

                    x_2d = int((x_3d / z_3d) * focal + center_x)
                    y_2d = int((y_3d / z_3d) * focal + center_y)

                    # 绘制标记
                    cv2.circle(display_image, (x_2d, y_2d), 8, (0, 0, 255), -1)
                    cv2.circle(display_image, (x_2d, y_2d), 12, (0, 0, 255), 2)

                    # 绘制连线
                    cv2.line(display_image, (center_x, center_y), (x_2d, y_2d), (255, 0, 0), 2)

                    # 显示信息
                    info_text = "ID:%d D:%.1fm" % (mark['id'], mark['distance'])
                    cv2.putText(display_image, info_text, (x_2d + 10, y_2d),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            except:
                continue

        # 显示状态信息
        status = "Marks: %d" % len(marks) if marks else "No marks"
        cv2.putText(display_image, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return display_image

    def real_time_detection(self, camera_id=0):
        """
        实时Mark检测与视频显示
        :param camera_id: 0=顶部摄像头, 1=底部摄像头
        """
        print "启动实时Mark检测..."
        print "按 'q' 退出，按 's' 截图"

        # 设置摄像头
        if not self.setup_camera(camera_id):
            print "摄像头初始化失败"
            return

        # 启动Mark检测
        self.start_detection()

        frame_count = 0
        start_time = time.time()

        try:
            while True:
                # 获取图像
                image = self.get_camera_image()
                if image is None:
                    continue

                frame_count += 1

                # 获取Mark数据
                raw_data = self.get_mark_data()
                marks = self.parse_mark_data_simple(raw_data)

                # 绘制检测结果
                display_image = self.draw_detection_info(image, marks)

                if display_image is not None:
                    # 计算FPS
                    current_time = time.time()
                    fps = frame_count / (current_time - start_time) if current_time > start_time else 0

                    # 显示FPS
                    fps_text = "FPS: %.1f" % fps
                    cv2.putText(display_image, fps_text, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # 显示图像
                    cv2.imshow('NAO Mark Detection - Press Q to quit', display_image)

                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q或ESC退出
                    break
                elif key == ord('s'):  # S截图
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = "nao_screenshot_%s.jpg" % timestamp
                    cv2.imwrite(filename, image)
                    print "截图已保存: %s" % filename
                elif key == ord('d'):  # D显示调试信息
                    print "检测到 %d 个Mark" % len(marks)
                    for mark in marks:
                        print "  Mark ID: %d, 位置: (%.2f, %.2f, %.2f), 距离: %.2fm" % (
                            mark['id'], mark['x'], mark['y'], mark['z'], mark['distance'])

        except KeyboardInterrupt:
            print "检测被用户中断"
        except Exception as e:
            print "检测出错: %s" % e
        finally:
            # 清理资源
            self.stop_detection()
            self.cleanup_camera()
            cv2.destroyAllWindows()
            print "实时检测结束"


def main():
    """主函数"""
    # 设置你的NAO机器人IP
    ROBOT_IP = "192.168.31.63"

    # 创建检测器
    detector = NAOMarkDetector(ROBOT_IP)

    # 开始实时检测（使用顶部摄像头）
    detector.real_time_detection(camera_id=0)


if __name__ == "__main__":
    main()