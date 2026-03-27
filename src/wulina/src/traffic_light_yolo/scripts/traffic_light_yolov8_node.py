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
from ultralytics import YOLO
from std_msgs.msg import String # 修改部分：导入 String 消息类型

class TrafficLightDetector:

    def __init__(self):
        rospy.init_node('traffic_light_yolov8_node')

        # --- 新增部分：摄像头参数 ---
        # !!! 重要: 请根据您的摄像头型号修改此参数 !!!
        # 这是一个估计值，常见的USB摄像头水平视场角在60-70度之间
        self.HORIZONTAL_FOV = 65.0 
        self.image_width = 0 # 图像宽度，将在第一帧时获取
        # ----------------------------

        # 模型路径
        model_path = "/home/gdut/catkin_roscar/src/wulina/src/traffic_light_yolo/best_new.pt"

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

        # --- 修改部分：创建新的发布器 ---
        # 发布到 /traffic_light_info 话题，消息类型为 String
        self.info_pub = rospy.Publisher('/traffic_light_info', String, queue_size=1)
        # -------------------------------

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")
        rospy.loginfo(f"Using estimated Horizontal FOV: {self.HORIZONTAL_FOV} degrees.")

    # --- 修改部分：函数现在返回标签和检测框 ---
    def detect_light(self, results):
        """
        在检测结果中查找最重要的红绿灯（红色优先）。
        返回: (label, box) 元组。例如 ('red', [x1, y1, x2, y2]) 或 ('none', None)
        """
        detected_label = "none"
        detected_box = None

        if len(results[0].boxes) == 0:
            return detected_label, detected_box

        names = results[0].names
        
        # 遍历所有检测结果
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]

            # 红色信号具有最高优先级
            if label == "red":
                return "red", box.xyxy[0].cpu().numpy() # 返回标签和检测框坐标

            # 如果没有红色，则记录绿色信号
            if label == "green":
                detected_label = "green"
                detected_box = box.xyxy[0].cpu().numpy()

        return detected_label, detected_box
    # ---------------------------------------------

    # --- 新增部分：角度计算函数 ---
    def calculate_angle(self, box_center_x):
        """
        根据物体在图像中的中心x坐标计算其水平角度。
        返回: 角度（度），右侧为正，左侧为负。
        """
        if self.image_width == 0:
            return 0.0
            
        # 图像中心x坐标
        image_center_x = self.image_width / 2.0
        
        # 物体中心与图像中心的像素偏差
        pixel_offset = box_center_x - image_center_x
        
        # 计算角度
        # (pixel_offset / image_center_x) 得到一个从-1到1的比例
        # 乘以半个视场角，得到实际角度
        angle = (pixel_offset / image_center_x) * (self.HORIZONTAL_FOV / 2.0)
        
        return angle
    # ----------------------------

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # --- 新增部分：在第一帧获取图像宽度 ---
        if self.image_width == 0:
            self.image_width = frame.shape[1]
            rospy.loginfo(f"Image width detected: {self.image_width} pixels.")
        # ------------------------------------

        # YOLO推理
        results = self.model(frame, verbose=False, device=0)

        # --- 修改部分：处理检测结果、计算角度并发布 ---
        label, box = self.detect_light(results)
        
        angle_deg = 0.0
        
        if label != "none" and box is not None:
            # 计算检测框的中心x坐标
            box_center_x = (box[0] + box[2]) / 2.0
            # 计算角度
            angle_deg = self.calculate_angle(box_center_x)

        # 格式化消息字符串: "颜色,角度"
        info_string = f"{label},{angle_deg:.1f}"
        
        # 发布消息
        self.info_pub.publish(info_string)
        # ---------------------------------------------

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0

        # 修改日志输出，显示更详细的信息
        rospy.loginfo(f"Published: '{info_string}' | FPS: {fps:.2f}")


    def shutdown(self):
        print("\n==============================")
        print("Traffic light detection node shut down.")
        print("==============================")


if __name__ == '__main__':
    detector = TrafficLightDetector()
    rospy.on_shutdown(detector.shutdown)
    rospy.spin()
