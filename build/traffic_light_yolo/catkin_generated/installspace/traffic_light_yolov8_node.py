#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["LD_PRELOAD"]="/usr/lib/aarch64-linux-gnu/libgomp.so.1"

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO # type: ignore

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        # 模型路径
        model_path = "/home/gdut/catkin_roscar/src/traffic_light_yolo/best.pt"

        # 加载YOLOv8模型
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 订阅图像
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # txt文件
        self.file = open("traffic_light_result.txt", "w")

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    def detect_light(self, results):

        label = "none"

        if len(results[0].boxes) == 0:
            return label

        names = results[0].names

        for cls in results[0].boxes.cls:
            name = names[int(cls)]

            if name == "red":
                return "red"

            if name == "green":
                label = "green"

        return label

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # YOLO推理
        results = self.model(frame, verbose=False,device=0)

        # 判断红绿灯
        result_label = self.detect_light(results)

        # 写入txt
        self.file.write(result_label + "\n")

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        rospy.loginfo("Result: %s   FPS: %.2f", result_label, fps)

    def shutdown(self):

        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        print("\n==============================")
        print("Total frames:", self.frame_count)
        print("Average FPS:", fps)
        print("==============================")

        self.file.close()


if __name__ == '__main__':

    detector = TrafficLightDetector()

    rospy.on_shutdown(detector.shutdown)

    rospy.spin()