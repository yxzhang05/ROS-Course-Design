#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
odom_navi.py — 纯里程计 PID 坐标导航节点
==========================================
以节点启动时的机器人位置作为坐标原点，根据用户输入的目标坐标 (x, y)，
使用 PID 算法驱动三轮差速机器人移动到指定位置，全程无需地图或激光雷达。

算法流程（状态机）
------------------
  IDLE      → 等待用户输入目标坐标
  ROTATING  → 原地旋转，使机器人朝向目标方向（角度 PID，线速度 = 0）
  MOVING    → 直线前进到目标点（线速度 PID + 角度修正 PID）
  DONE      → 到达目标点，停车，等待下一个目标

传感器使用
----------
  /odom  (nav_msgs/Odometry) : 读取机器人在里程计坐标系中的位置 (x, y) 与偏航角 (yaw)
  /imu   (sensor_msgs/Imu)   : 可选。若话题可用，使用 IMU 的偏航角替换里程计偏航角，
                               提供更精确的朝向信息（陀螺仪漂移较小）

速度限制
--------
  最大线速度  : 0.1 m/s（可通过参数 ~max_linear_vel 调整）
  最大角速度  : 1.0 rad/s（可通过参数 ~max_angular_vel 调整）

ROS 参数（均可在 launch 文件或命令行覆盖）
------------------------------------------
  ~max_linear_vel   (float, 默认 0.10)  — 最大线速度 (m/s)
  ~max_angular_vel  (float, 默认 1.00)  — 最大角速度 (rad/s)
  ~dist_tolerance   (float, 默认 0.05)  — 到达目标点的距离容差 (m)
  ~angle_tolerance  (float, 默认 0.05)  — 旋转对准的角度容差 (rad)
  ~control_freq     (float, 默认 20.0)  — 控制循环频率 (Hz)
  ~use_imu_yaw      (bool,  默认 True)  — 是否优先使用 IMU 偏航角
  ~lin_kp           (float, 默认 0.50)  — 线速度 PID 比例系数
  ~lin_ki           (float, 默认 0.00)  — 线速度 PID 积分系数
  ~lin_kd           (float, 默认 0.08)  — 线速度 PID 微分系数
  ~ang_kp           (float, 默认 2.00)  — 角速度 PID 比例系数
  ~ang_ki           (float, 默认 0.00)  — 角速度 PID 积分系数
  ~ang_kd           (float, 默认 0.20)  — 角速度 PID 微分系数
  ~heading_correct_threshold (float, 默认 0.35) — 前进中朝向偏差超过此值
                              时停车重新旋转对准 (rad)
"""

import math
import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


# ─────────────────────────────────────────────
#  PID 控制器
# ─────────────────────────────────────────────

class PID:
    """简单 PID 控制器（带输出限幅与积分限幅）。"""

    def __init__(self, kp, ki, kd, out_max, out_min=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.out_min = out_min if out_min is not None else -out_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def compute(self, error):
        """根据误差计算控制输出，返回限幅后的值。"""
        now = rospy.Time.now().to_sec()
        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time
            dt = max(dt, 0.0)
        self._last_time = now

        self._integral += error * dt
        # 积分限幅，防止积分饱和
        self._integral = max(self.out_min / (self.ki + 1e-9),
                             min(self.out_max / (self.ki + 1e-9),
                                 self._integral))

        derivative = (error - self._prev_error) / dt if dt > 1e-6 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


# ─────────────────────────────────────────────
#  导航节点
# ─────────────────────────────────────────────

class OdomNavi:
    # 状态机枚举
    IDLE     = 'IDLE'
    ROTATING = 'ROTATING'
    MOVING   = 'MOVING'
    DONE     = 'DONE'

    def __init__(self):
        rospy.init_node('odom_navi', anonymous=False)

        # ── 读取 ROS 参数 ──────────────────────────────────────────
        self.max_linear_vel  = rospy.get_param('~max_linear_vel',  0.10)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.00)
        self.dist_tolerance  = rospy.get_param('~dist_tolerance',  0.05)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.05)
        self.control_freq    = rospy.get_param('~control_freq',    20.0)
        self.use_imu_yaw     = rospy.get_param('~use_imu_yaw',     True)
        self.heading_correct_threshold = rospy.get_param(
            '~heading_correct_threshold', 0.35)

        lin_kp = rospy.get_param('~lin_kp', 0.50)
        lin_ki = rospy.get_param('~lin_ki', 0.00)
        lin_kd = rospy.get_param('~lin_kd', 0.08)
        ang_kp = rospy.get_param('~ang_kp', 2.00)
        ang_ki = rospy.get_param('~ang_ki', 0.00)
        ang_kd = rospy.get_param('~ang_kd', 0.20)

        self.linear_pid  = PID(lin_kp, lin_ki, lin_kd, self.max_linear_vel)
        self.angular_pid = PID(ang_kp, ang_ki, ang_kd, self.max_angular_vel)

        # ── 状态变量（受互斥锁保护）────────────────────────────────
        self._lock = threading.Lock()

        # 里程计原点（节点启动后首次收到 odom 时记录）
        self._origin_x = None
        self._origin_y = None

        # 当前位姿（相对原点坐标系）
        self._cur_x   = 0.0
        self._cur_y   = 0.0
        self._cur_yaw = 0.0  # 偏航角（单位：rad），来自里程计或 IMU

        # IMU 偏航角状态
        self._imu_yaw_ready = False  # 是否已收到过 IMU 数据
        self._imu_yaw       = 0.0   # 最新 IMU 偏航角（绝对值）
        self._imu_yaw_origin = None  # 原点时刻的 IMU 偏航角（用于增量计算）

        # 目标点
        self._goal_x   = 0.0
        self._goal_y   = 0.0
        self._state    = self.IDLE

        # ── ROS 通信 ───────────────────────────────────────────────
        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb,
                                         queue_size=5)
        self.imu_sub  = rospy.Subscriber('/imu', Imu, self._imu_cb,
                                          queue_size=5)

        rospy.on_shutdown(self._on_shutdown)

        # ── 等待第一帧里程计，确定坐标原点 ─────────────────────────
        rospy.loginfo('[odom_navi] 等待里程计数据...')
        try:
            rospy.wait_for_message('/odom', Odometry, timeout=10.0)
        except rospy.ROSException:
            rospy.logfatal('[odom_navi] 超时：未收到 /odom，请检查底盘驱动是否正常运行。')
            rospy.signal_shutdown('no odom')
            return

        with self._lock:
            rospy.loginfo('[odom_navi] 坐标原点已确定 → 里程计 (%.3f, %.3f)',
                          self._origin_x, self._origin_y)

        # 等待 IMU（非阻塞，1 秒内收到则启用）
        if self.use_imu_yaw:
            try:
                rospy.wait_for_message('/imu', Imu, timeout=1.0)
                rospy.loginfo('[odom_navi] IMU 可用，使用 IMU 偏航角')
            except rospy.ROSException:
                self.use_imu_yaw = False
                rospy.logwarn('[odom_navi] 未收到 /imu，使用里程计偏航角')

        self._rate = rospy.Rate(self.control_freq)

    # ================================================================
    #  话题回调
    # ================================================================

    def _odom_cb(self, msg):
        with self._lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # 首次收到：记录原点
            if self._origin_x is None:
                self._origin_x = x
                self._origin_y = y

            # 相对坐标
            self._cur_x = x - self._origin_x
            self._cur_y = y - self._origin_y

            # 若 IMU 不可用，使用里程计偏航角
            if not (self.use_imu_yaw and self._imu_yaw_ready):
                q = msg.pose.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self._cur_yaw = yaw

    def _imu_cb(self, msg):
        if not self.use_imu_yaw:
            return
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self._lock:
            # 首次：记录初始 IMU 偏航角（用于计算增量）
            if self._imu_yaw_origin is None:
                self._imu_yaw_origin = yaw
            self._imu_yaw = yaw
            self._imu_yaw_ready = True
            # IMU 偏航角增量 + 原点时的里程计偏航角，保持坐标系一致
            self._cur_yaw = yaw - self._imu_yaw_origin

    # ================================================================
    #  主控制循环（在主线程中运行）
    # ================================================================

    def run(self):
        """主循环：PID 控制 + 等待用户输入。"""
        # 用户输入线程
        input_thread = threading.Thread(target=self._input_loop, daemon=True)
        input_thread.start()

        rospy.loginfo('[odom_navi] 节点就绪。请在终端输入目标坐标。')
        self._print_prompt()

        while not rospy.is_shutdown():
            with self._lock:
                state  = self._state
                cur_x  = self._cur_x
                cur_y  = self._cur_y
                yaw    = self._cur_yaw
                goal_x = self._goal_x
                goal_y = self._goal_y

            twist = Twist()

            if state == self.ROTATING:
                twist = self._control_rotating(cur_x, cur_y, yaw,
                                               goal_x, goal_y)

            elif state == self.MOVING:
                twist = self._control_moving(cur_x, cur_y, yaw,
                                             goal_x, goal_y)

            # IDLE / DONE：发布零速，保持停止
            self.cmd_pub.publish(twist)
            self._rate.sleep()

    # ── 旋转阶段控制 ──────────────────────────────────────────────

    def _control_rotating(self, cur_x, cur_y, yaw, goal_x, goal_y):
        """原地旋转，使机器人朝向目标方向。"""
        target_yaw = math.atan2(goal_y - cur_y, goal_x - cur_x)
        err = _normalize_angle(target_yaw - yaw)

        if abs(err) < self.angle_tolerance:
            # 对准完成，切换到前进阶段
            with self._lock:
                self._state = self.MOVING
            self.angular_pid.reset()
            self.linear_pid.reset()
            rospy.loginfo('[odom_navi] 对准方向完成（误差 %.3f rad），开始前进…', err)
            return Twist()

        twist = Twist()
        twist.angular.z = self.angular_pid.compute(err)
        return twist

    # ── 前进阶段控制 ──────────────────────────────────────────────

    def _control_moving(self, cur_x, cur_y, yaw, goal_x, goal_y):
        """线速度 PID 前进 + 角度修正。"""
        dx   = goal_x - cur_x
        dy   = goal_y - cur_y
        dist = math.hypot(dx, dy)

        if dist < self.dist_tolerance:
            # 到达目标点
            with self._lock:
                self._state = self.DONE
            rospy.loginfo('[odom_navi] ✓ 已到达目标点 (%.3f, %.3f)，'
                          '实际位置 (%.3f, %.3f)，误差 %.3f m',
                          goal_x, goal_y, cur_x, cur_y, dist)
            sys.stdout.write('\n[odom_navi] 已到达目标点。\n')
            sys.stdout.flush()
            self._print_prompt()
            return Twist()

        target_yaw  = math.atan2(dy, dx)
        heading_err = _normalize_angle(target_yaw - yaw)

        # 朝向偏差过大时停车重新旋转
        if abs(heading_err) > self.heading_correct_threshold:
            with self._lock:
                self._state = self.ROTATING
            self.linear_pid.reset()
            self.angular_pid.reset()
            rospy.logwarn('[odom_navi] 朝向偏差 %.2f rad 超限，停车重新对准…',
                          heading_err)
            return Twist()

        twist = Twist()
        twist.linear.x  = self.linear_pid.compute(dist)
        twist.angular.z = self.angular_pid.compute(heading_err)
        return twist

    # ================================================================
    #  用户输入线程
    # ================================================================

    def _input_loop(self):
        """在独立线程中循环读取用户输入的目标坐标。"""
        while not rospy.is_shutdown():
            try:
                raw = input()
            except EOFError:
                break

            raw = raw.strip()
            if not raw:
                continue

            if raw.lower() in ('q', 'quit', 'exit'):
                rospy.loginfo('[odom_navi] 用户请求退出。')
                rospy.signal_shutdown('user quit')
                break

            # 解析 "x y" 格式
            parts = raw.replace(',', ' ').split()
            if len(parts) < 2:
                sys.stdout.write('[odom_navi] 格式错误，请输入: x y（例如 1.5 0.8）\n')
                sys.stdout.flush()
                self._print_prompt()
                continue

            try:
                gx = float(parts[0])
                gy = float(parts[1])
            except ValueError:
                sys.stdout.write('[odom_navi] 无法解析数字，请重新输入。\n')
                sys.stdout.flush()
                self._print_prompt()
                continue

            with self._lock:
                # 如果当前正在执行，先停车再设置新目标
                self._goal_x = gx
                self._goal_y = gy
                self._state  = self.ROTATING
            self.linear_pid.reset()
            self.angular_pid.reset()

            with self._lock:
                cx, cy = self._cur_x, self._cur_y
            dist = math.hypot(gx - cx, gy - cy)
            rospy.loginfo('[odom_navi] 新目标: (%.3f, %.3f)，当前位置: (%.3f, %.3f)，'
                          '距离: %.3f m', gx, gy, cx, cy, dist)

    def _print_prompt(self):
        with self._lock:
            cx, cy = self._cur_x, self._cur_y
        sys.stdout.write(
            '\n当前位置: ({:.3f}, {:.3f})  '
            '输入目标坐标 "x y"，或输入 "q" 退出: '.format(cx, cy))
        sys.stdout.flush()

    # ================================================================
    #  安全停车
    # ================================================================

    def _on_shutdown(self):
        rospy.logwarn('[odom_navi] 节点关闭，发送停车指令…')
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)
        rospy.loginfo('[odom_navi] 小车已停止。')


# ─────────────────────────────────────────────
#  工具函数
# ─────────────────────────────────────────────

def _normalize_angle(angle):
    """将角度规范化到 (-π, π]。"""
    while angle >  math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


# ─────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────

if __name__ == '__main__':
    try:
        node = OdomNavi()
        node.run()
    except rospy.ROSInterruptException:
        pass
