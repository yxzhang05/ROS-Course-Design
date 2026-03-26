#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["LD_PRELOAD"] = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String


class TrafficLightDetector:

    def __init__(self):
        rospy.init_node('traffic_light_yolov8_node')

        # 摄像头水平视场角（度）——请根据实际摄像头型号修改
        self.HORIZONTAL_FOV = 65.0
        self.image_width = 0  # 首帧时自动获取

        # YOLOv8 模型文件路径（通过 ROS 参数服务器配置，方便不同机器适配）
        # 命令行覆盖：rosrun traffic_light_yolo traffic_light_yolov8_node.py
        #             _model_path:=/path/to/best.pt
        default_model = "/home/gdut/catkin_roscar/src/traffic_light_yolo/best_new.pt"
        model_path = rospy.get_param('~model_path', default_model)

        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 订阅摄像头图像
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        # 发布识别结果：格式为 "颜色,角度"，例如 "red,5.2" / "green,-3.1" / "none,0.0"
        self.info_pub = rospy.Publisher('/traffic_light_info', String, queue_size=1)

        # FPS 统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")
        rospy.loginfo("Estimated Horizontal FOV: %.1f degrees.", self.HORIZONTAL_FOV)

    def detect_light(self, results):
        """
        在检测结果中查找最重要的红绿灯（红色优先）。
        返回: (label, box) 元组，例如 ('red', [x1,y1,x2,y2]) 或 ('none', None)。
        """
        detected_label = "none"
        detected_box = None

        if len(results[0].boxes) == 0:
            return detected_label, detected_box

        names = results[0].names

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]

            if label == "red":
                return "red", box.xyxy[0].cpu().numpy()

            if label == "green":
                detected_label = "green"
                detected_box = box.xyxy[0].cpu().numpy()

        return detected_label, detected_box

    def calculate_angle(self, box_center_x):
        """
        根据检测框中心 x 坐标计算水平角度（度）。
        右侧为正，左侧为负。
        """
        if self.image_width == 0:
            return 0.0
        image_center_x = self.image_width / 2.0
        pixel_offset = box_center_x - image_center_x
        angle = (pixel_offset / image_center_x) * (self.HORIZONTAL_FOV / 2.0)
        return angle

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return

        if self.image_width == 0:
            self.image_width = frame.shape[1]
            rospy.loginfo("Image width: %d pixels.", self.image_width)

        results = self.model(frame, verbose=False, device=0)

        label, box = self.detect_light(results)

        angle_deg = 0.0
        if label != "none" and box is not None:
            box_center_x = (box[0] + box[2]) / 2.0
            angle_deg = self.calculate_angle(box_center_x)

        info_string = "{},{:.1f}".format(label, angle_deg)
        self.info_pub.publish(info_string)

        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        rospy.loginfo("Published: '%s' | FPS: %.2f", info_string, fps)

    def shutdown(self):
        print("\n==============================")
        print("Traffic light detection node shut down.")
        print("==============================")


if __name__ == '__main__':
    detector = TrafficLightDetector()
    rospy.on_shutdown(detector.shutdown)
    rospy.spin()
