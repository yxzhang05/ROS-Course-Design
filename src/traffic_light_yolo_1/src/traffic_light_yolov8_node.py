#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["LD_PRELOAD"]="/usr/lib/aarch64-linux-gnu/libgomp.so.1"

import math
import collections
import time
import statistics

import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from ultralytics import YOLO # type: ignore
from std_msgs.msg import Float32MultiArray


class TrafficLightDetector:

    def __init__(self):
        rospy.init_node('traffic_light_yolov8_node')

        # 模型
        model_path = rospy.get_param("~model_path", "/home/gdut/catkin_roscar/src/traffic_light_yolo_1/best_new.pt")
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 摄像头参数
        self.image_width = None
        self.camera_fov  = rospy.get_param("~camera_fov", 93.4)  # 水平FOV（度）

        # 摄像头与雷达安装偏角补偿（度），实测后调整
        self.angle_offset = rospy.get_param("~angle_offset", 0.0)

        # 去抖参数
        debounce_window         = rospy.get_param("~debounce_window",    10)
        self.debounce_threshold = rospy.get_param("~debounce_threshold", 8)
        self.raw_history        = collections.deque(maxlen=debounce_window)

        # 确认后的灯态
        self.confirmed_state = 0    # 0=none 1=red 2=green
        self.confirmed_angle = 0.0  # 度

        # 角度低通滤波
        self.angle_filter_alpha = rospy.get_param("~angle_filter_alpha", 0.3)  # 新权重，越小越平滑
        self.filtered_angle = 0.0
        self.angle_filter_initialized = False

        # 距离滤波
        self.distance_window_size = rospy.get_param("~distance_window_size", 5)
        self.distance_history = collections.deque(maxlen=self.distance_window_size)
        self.max_distance_jump = rospy.get_param("~max_distance_jump", 1)  # 米，相邻帧最大允许跳变
        self.last_filtered_distance = None  # 上一帧真实输出值，用于跳变限幅

        # 检测丢失缓冲
        self.max_lost_frames = rospy.get_param("~max_lost_frames", 3)  # 允许连续丢失的最大帧数
        self.lost_count = 0  # 当前连续丢失计数

        # 时间同步
        self.max_time_diff = rospy.get_param("~max_time_diff", 0.1)  # 秒，图像与雷达最大允许时差
        self.last_image_stamp = None

        # 雷达
        self.scan = None
        rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=1)

        # 发布
        # 格式: [state(0=none,1=red,2=green), angle(度), distance(m)]
        self.detection_pub = rospy.Publisher(
            "/traffic_light_detection", Float32MultiArray, queue_size=1)

        # 订阅图像
        rospy.Subscriber("/usb_cam/image_raw", Image,
                         self.image_callback, queue_size=1, buff_size=2**24)

        # txt记录
        self.file = open("traffic_light_result.txt", "w")

        # FPS
        self.frame_count = 0
        self.start_time  = time.time()

        # 距离打印计时
        self.last_print_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    # ----------------------------------------------------------
    # 回调：雷达
    # ----------------------------------------------------------
    def _scan_cb(self, msg):
        self.scan = msg

    # ----------------------------------------------------------
    # 测距
    # ----------------------------------------------------------
    def _calc_distance(self, angle_deg):
        """用视觉角度查雷达对应索引处的距离（单点精准测量），返回距离（m），无有效点返回 inf"""
        if self.scan is None:
            return float('inf')
        # 时间同步检查
        if self.last_image_stamp is not None:
            time_diff = abs((self.last_image_stamp - self.scan.header.stamp).to_sec())
            if time_diff > self.max_time_diff:
                rospy.logwarn_throttle(1.0, "Image-LaserScan time diff too large: %.3fs > %.3fs",
                                       time_diff, self.max_time_diff)
                return float('inf')
        scan = self.scan
        angle_rad = math.radians(angle_deg + self.angle_offset)
        n = len(scan.ranges)

        # 计算对应索引
        index = int((angle_rad - scan.angle_min) / scan.angle_increment)

        # 检查索引是否在有效范围内
        if index < 0 or index >= n:
            return float('inf')

        distance = scan.ranges[index]

        # 过滤无效值（包括 inf/nan 以及超出雷达量程的值）
        if (math.isinf(distance) or math.isnan(distance) or
                not (scan.range_min <= distance <= scan.range_max)):
            return float('inf')

        return distance

    # ----------------------------------------------------------
    # 距离滤波
    # ----------------------------------------------------------
    def _filter_distance(self, raw_distance):
        """对原始距离进行滤波：限幅跳变 + 滑动窗口中值"""
        # 无效距离直接入历史，不做跳变限幅
        if raw_distance == float('inf'):
            self.distance_history.append(float('inf'))
            self.last_filtered_distance = None  # 无效帧重置，下帧重新初始化
            return float('inf')

        # 限幅跳变：与上一帧真实输出值比较
        if self.last_filtered_distance is not None:
            if abs(raw_distance - self.last_filtered_distance) > self.max_distance_jump:
                filtered = self.last_filtered_distance  # 跳变过大，冻结
            else:
                filtered = raw_distance
        else:
            # 初始化时，检查历史窗口中的有效值，防止从inf恢复时跳变过大
            if self.distance_history:
                recent_valid = [d for d in self.distance_history if d != float('inf')]
                if recent_valid:
                    historical_median = statistics.median(recent_valid)
                    if abs(raw_distance - historical_median) > self.max_distance_jump:
                        # 跳变过大，使用历史中值
                        filtered = historical_median
                        rospy.logdebug("Distance jump too large on initialization: raw=%.2f, historical=%.2f, using historical",
                                     raw_distance, historical_median)
                    else:
                        filtered = raw_distance
                else:
                    filtered = raw_distance
            else:
                filtered = raw_distance

        # 加入历史
        self.distance_history.append(filtered)

        # 滑动窗口中值滤波
        valid_recent = [d for d in self.distance_history if d != float('inf')]
        if not valid_recent:
            return float('inf')

        result = statistics.median(valid_recent)
        self.last_filtered_distance = result  # 保存本帧输出值供下帧使用
        return result

    # ----------------------------------------------------------
    # 检测
    # ----------------------------------------------------------
    def detect_light(self, results):
        """返回置信度最高的红/绿灯框 (label, cx, width)，未检测到返回 ("none", -1, -1)"""
        if len(results[0].boxes) == 0:
            return "none", -1.0, -1.0
        names = results[0].names
        boxes = results[0].boxes
        best_label, best_cx, best_w, best_conf = "none", -1.0, -1.0, -1.0
        for i, cls in enumerate(boxes.cls):
            name = names[int(cls)]
            if name in ("red", "green"):
                conf = float(boxes.conf[i])
                if conf > best_conf:
                    box = boxes.xywh[i]
                    best_label, best_cx, best_w, best_conf = name, float(box[0]), float(box[2]), conf
        return best_label, best_cx, best_w

    # ----------------------------------------------------------
    # 图像回调
    # ----------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("imgmsg_to_cv2 failed: %s", e)
            return

        self.last_image_stamp = msg.header.stamp
        self.image_width = msg.width

        # 推理
        results = self.model(frame, verbose=False, device=0)
        result_label, cx, bbox_width = self.detect_light(results)

        # 角度（ROS坐标系：左正右负）
        if cx >= 0:
            raw_angle = -(cx - self.image_width / 2.0) * self.camera_fov / self.image_width
            # 角度低通滤波
            if not self.angle_filter_initialized:
                self.filtered_angle = raw_angle
                self.angle_filter_initialized = True
            else:
                self.filtered_angle = (self.angle_filter_alpha * raw_angle +
                                      (1.0 - self.angle_filter_alpha) * self.filtered_angle)
            angle = self.filtered_angle
        else:
            angle = 0.0

        # 去抖
        state_map = {"none": 0, "red": 1, "green": 2}
        self.raw_history.append(state_map[result_label])
        red_count   = self.raw_history.count(1)
        green_count = self.raw_history.count(2)

        # 保存之前的状态
        prev_state = self.confirmed_state

        # 计算候选状态（基于原始统计）
        candidate_state = 0
        candidate_angle = 0.0
        if red_count >= self.debounce_threshold:
            candidate_state = 1
            candidate_angle = angle
        elif green_count >= self.debounce_threshold:
            candidate_state = 2
            candidate_angle = angle

        # 应用检测丢失缓冲
        if candidate_state != 0:
            # 检测到有效状态
            self.confirmed_state = candidate_state
            self.confirmed_angle = candidate_angle
            self.lost_count = 0  # 重置丢失计数
        elif prev_state != 0:
            # 之前有有效状态，但当前帧统计不支持
            self.lost_count += 1
            if self.lost_count < self.max_lost_frames:
                # 保持之前状态（不更新角度）
                rospy.logdebug("Light detection lost, count=%d/%d, keeping state %d",
                             self.lost_count, self.max_lost_frames, prev_state)
            else:
                # 连续丢失超过阈值，重置状态
                self.confirmed_state = 0
                self.confirmed_angle = 0.0
                self.lost_count = 0
                rospy.logdebug("Light detection lost for %d frames, resetting state", self.max_lost_frames)
        else:
            # 之前就是none状态
            self.confirmed_state = 0
            self.confirmed_angle = 0.0
            self.lost_count = 0

        # 测距（只在检测到灯时测）
        if self.confirmed_state != 0:
            raw_distance = self._calc_distance(self.confirmed_angle)
            # 距离滤波：限幅跳变 + 滑动窗口中值
            distance = self._filter_distance(raw_distance)
        else:
            # 灯消失时也更新 history，避免残留旧值影响下次跳变判断
            self._filter_distance(float('inf'))
            distance = float('inf')

        # 发布 [state, angle, distance]
        msg_out = Float32MultiArray()
        msg_out.data = [float(self.confirmed_state), self.confirmed_angle, distance]
        self.detection_pub.publish(msg_out)

        # 写txt
        label_map = {0: "none", 1: "red", 2: "green"}
        self.file.write(label_map[self.confirmed_state] + "\n")

        # 每秒打印一次灯态+距离
        now = time.time()
        if now - self.last_print_time >= 1.0:
            dist_str = "{:.2f} m".format(distance) if distance != float('inf') else "inf"
            rospy.loginfo("Light: %-5s  Distance: %s  FPS: %.1f",
                          label_map[self.confirmed_state], dist_str,
                          self.frame_count / (now - self.start_time))
            self.last_print_time = now

        self.frame_count += 1

    def shutdown(self):
        elapsed = time.time() - self.start_time
        print("\n==============================")
        print("Total frames:", self.frame_count)
        print("Average FPS:  {:.2f}".format(self.frame_count / elapsed))
        print("==============================")
        self.file.close()


if __name__ == '__main__':
    detector = TrafficLightDetector()
    rospy.on_shutdown(detector.shutdown)
    rospy.spin()
