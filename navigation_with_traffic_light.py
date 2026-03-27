#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import signal
import os
import time
import math

# --- 新增部分：导入新的消息类型 ---
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
# -----------------------------------

class NavigationController:
    """
    导航控制节点（智能决策版）
    ========================
    功能：
    1. 订阅红绿灯检测节点发布的颜色和角度信息 (/traffic_light_info)
    2. 订阅激光雷达数据 (/scan)
    3. 融合数据，计算红绿灯距离
    4. 实时在终端显示状态和距离
    5. 决策：
       - 红灯：通过话题控制导航节点暂停
       - 绿灯：通过话题控制导航节点恢复
    """
    
    def __init__(self):
        rospy.init_node('navigation_controller', anonymous=False)
        
        # 导航进程
        self.nav_process = None
        self.navigation_paused = False
        self.nav_launch_cmd = ['roslaunch', 'wulina', 'multi_point_navigation.launch']
        
        # --- 状态和参数定义 ---
        self.lidar_data = None
        self.current_light_state = "none"  # 'red', 'green', 或 'none'
        self.current_light_angle_deg = 0.0 # 角度（度）
        self.last_detection_time = rospy.Time.now()
        
        # 行为参数
        self.stop_distance = 1.0  # 目标停止距离 (米)
        self.action_threshold = self.stop_distance + 0.5 # 提前响应的距离 (米)，给刹车留出余量
        self.detection_timeout = rospy.Duration(2.0) # 如果2秒没收到信号，则认为没有红绿灯
        # ------------------------------------

        # --- 新增：发布暂停控制信号 ---
        self.pause_pub = rospy.Publisher('/navigation_pause', Bool, queue_size=1)
        # --------------------------------

        # 查找已运行的导航进程
        self.find_navigation_process()
        
        # --- 订阅话题 ---
        rospy.Subscriber('/traffic_light_info', String, self.light_info_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        # ----------------------------

        # --- 定时器用于主循环 ---
        rospy.Timer(rospy.Duration(0.5), self.decision_loop)
        # -----------------------------------
        
        rospy.loginfo("="*60)
        rospy.loginfo("Navigation Controller Started (Smart Decision Mode with Topic Control)")
        rospy.loginfo("  Subscribing to: /traffic_light_info (String)")
        rospy.loginfo("  Subscribing to: /scan (LaserScan)")
        rospy.loginfo(f"  Publishing to: /navigation_pause (Bool)")
        rospy.loginfo(f"  Action Threshold: Stop if RED and distance < {self.action_threshold:.2f} m")
        rospy.loginfo("="*60)
    
    # --- 回调函数 ---
    def light_info_callback(self, msg):
        """接收红绿灯信息，格式为 '颜色,角度'"""
        try:
            parts = msg.data.split(',')
            if len(parts) == 2:
                self.current_light_state = parts[0]
                self.current_light_angle_deg = float(parts[1])
                self.last_detection_time = rospy.Time.now()
        except Exception as e:
            rospy.logwarn(f"Invalid format in /traffic_light_info: {msg.data}. Error: {e}")
            self.current_light_state = "none"

    def lidar_callback(self, msg):
        """接收并存储激光雷达数据"""
        self.lidar_data = msg
    # ----------------------------

    # --- 主决策循环 ---
    def decision_loop(self, event):
        """每0.5秒执行一次的主决策循环"""
        if self.lidar_data is None:
            return

        # 检查红绿灯信号是否超时
        if rospy.Time.now() - self.last_detection_time > self.detection_timeout:
            self.current_light_state = "none"

        distance_to_light = float('inf')

        # 如果检测到红绿灯，计算距离
        if self.current_light_state != "none":
            try:
                # 将角度从度转换为弧度，注意ROS中角度通常是逆时针为正
                # 假设摄像头的角度中线与激光雷达的0度对齐
                angle_rad = math.radians(-self.current_light_angle_deg) 

                # 找到该角度在激光雷达数据数组中的索引
                index = int((angle_rad - self.lidar_data.angle_min) / self.lidar_data.angle_increment)
                
                # 确保索引在有效范围内
                if 0 <= index < len(self.lidar_data.ranges):
                    distance_to_light = self.lidar_data.ranges[index]
                    # 过滤无效值
                    if math.isinf(distance_to_light) or math.isnan(distance_to_light):
                        distance_to_light = float('inf')
                
            except Exception as e:
                rospy.logwarn_throttle(5, f"Could not calculate distance to light: {e}")
        
        # 实时显示到终端
        state_display = self.current_light_state.upper() if self.current_light_state != "none" else "NONE"
        dist_display = f"{distance_to_light:.2f}m" if distance_to_light != float('inf') else "---"
        rospy.loginfo_throttle(2, f"State: [{state_display}] | Distance: [{dist_display}] | Navigation Paused: {self.navigation_paused}")

        # 决策逻辑：使用话题控制，不再杀死进程
        # 1. 红灯且距离近，且导航未被暂停 -> 发送暂停信号
        if self.current_light_state == "red" and distance_to_light < self.action_threshold:
            if not self.navigation_paused:
                rospy.logwarn("="*60)
                rospy.logwarn(f"🔴 RED LIGHT approaching (Dist: {distance_to_light:.2f}m). Sending PAUSE command.")
                rospy.logwarn("="*60)
                # 发送暂停信号
                self.pause_pub.publish(True)
                self.navigation_paused = True
                rospy.loginfo("Pause command sent to navigation node")
            
        # 2. 绿灯，且导航已暂停 -> 发送恢复信号
        elif self.current_light_state == "green" and self.navigation_paused:
            rospy.loginfo("="*60)
            rospy.loginfo("🟢 GREEN LIGHT detected. Sending RESUME command.")
            rospy.loginfo("="*60)
            # 发送恢复信号
            self.pause_pub.publish(False)
            self.navigation_paused = False
            rospy.loginfo("Resume command sent to navigation node")
        
        # 3. 无红绿灯信号或信号超时，且导航被暂停 -> 自动恢复
        elif self.current_light_state == "none" and self.navigation_paused:
            # 检查是否超过5秒没有检测到任何信号
            time_since_last = rospy.Time.now() - self.last_detection_time
            if time_since_last > rospy.Duration(5.0):
                rospy.loginfo("="*60)
                rospy.loginfo(f"No traffic light detected for {time_since_last.to_sec():.1f}s, auto-resuming navigation.")
                rospy.loginfo("="*60)
                self.pause_pub.publish(False)
                self.navigation_paused = False
    # ----------------------------

    # 以下函数 find_navigation_process, start_navigation, stop_navigation 保留用于兼容
    def find_navigation_process(self):
        """查找已运行的导航进程"""
        try:
            result = subprocess.run(['pgrep', '-f', 'roslaunch.*multi_point_navigation'], capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                pid = int(result.stdout.strip().split('\n')[0])
                self.nav_process = type('obj', (object,), {'pid': pid})()
                rospy.loginfo(f"✓ Found navigation process (PID: {pid})")
                return
            result = subprocess.run(['pgrep', '-f', 'move_base'], capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                pid = int(result.stdout.strip().split('\n')[0])
                self.nav_process = type('obj', (object,), {'pid': pid})()
                rospy.loginfo(f"✓ Found navigation process (move_base PID: {pid})")
                return
            rospy.logwarn("No running navigation process found!")
        except Exception as e:
            rospy.logwarn(f"Could not find navigation process: {e}")

    def start_navigation(self):
        """启动导航节点（备用方法，实际使用话题控制）"""
        if self.nav_process is not None:
            try:
                os.kill(self.nav_process.pid, 0)
                rospy.loginfo("Navigation already running")
                return
            except ProcessLookupError:
                self.nav_process = None
        rospy.loginfo("Starting multi-point navigation...")
        try:
            with open(os.devnull, 'w') as FNULL:
                self.nav_process = subprocess.Popen(self.nav_launch_cmd, stdout=FNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            rospy.loginfo(f"✓ Navigation started (PID: {self.nav_process.pid})")
            time.sleep(3)
        except Exception as e:
            rospy.logerr(f"Failed to start navigation: {e}")

    def stop_navigation(self):
        """停止导航节点（备用方法，实际使用话题控制）"""
        if self.nav_process is None:
            self.find_navigation_process()
            if self.nav_process is None:
                rospy.logwarn("No navigation process to stop")
                return
        try:
            os.killpg(os.getpgid(self.nav_process.pid), signal.SIGTERM)
            try:
                self.nav_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Navigation did not stop gracefully, force killing...")
                os.killpg(os.getpgid(self.nav_process.pid), signal.SIGKILL)
                self.nav_process.wait(timeout=2)
            rospy.loginfo("✓ Navigation stopped")
        except ProcessLookupError:
            rospy.loginfo("Navigation already stopped")
        except Exception as e:
            rospy.logerr(f"Error stopping navigation: {e}")
        finally:
            self.nav_process = None

    def run(self):
        """主循环由定时器处理，rospy.spin()负责保持节点存活"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run()
    except rospy.ROSInterruptException:
        if 'controller' in locals() and controller.nav_process:
            controller.stop_navigation()
        rospy.loginfo("Navigation controller stopped")