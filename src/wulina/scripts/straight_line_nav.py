#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
直线导航节点（rotate-then-translate 版）
========================================
本节点实现以下导航原则：

  1. **只前进不后退** — 只向 /cmd_vel 发布 linear.x ≥ 0 的速度。
  2. **转弯与前进不同时进行** — 执行逻辑分为两个串行阶段：
       Phase 1: 原地旋转到面向目标点的方向（angular.z ≠ 0，linear.x = 0）
       Phase 2: 直线前进到目标点附近     （linear.x > 0，angular.z = 0）
       Phase 3: 原地旋转到目标点要求的最终朝向
     若前进途中朝向偏差超过 heading_correct_threshold（默认 0.20 rad ≈ 11.5°），
     则停止前进，重新原地旋转对准方向后继续前进，实现"停下-转-再走"修正。
  3. **无障碍物时规划直线路径** — 前进阶段直接走直线，不经过 move_base 规划。
     若前方激光扫描在检测角锥（默认 ±30°）内检测到距离小于 obstacle_dist 的
     障碍物，则停止直线模式，自动切换到 move_base 为该段进行绕障规划，完成后
     继续使用直线模式前往下一个航点。

参数说明（可通过 ROS 参数服务器或 launch 文件覆盖）：
  ~loop                      (bool,  默认 False)  — 是否循环执行航点列表
  ~loop_count                (int,   默认 1)      — 循环次数（loop=True 时有效）；0=无限
  ~goal_timeout              (float, 默认 60.0)   — 单个目标超时时间（秒）
  ~map_frame                 (str,   默认 "map")  — 地图坐标系名称
  ~base_frame                (str,   默认 "base_footprint") — 机器人本体坐标系
  ~interactive               (bool,  默认 True)   — 是否启用交互式终端控制
  ~linear_speed              (float, 默认 0.10)   — 直线前进速度（m/s）
  ~angular_speed             (float, 默认 0.30)   — 原地旋转角速度（rad/s）
  ~yaw_tolerance             (float, 默认 0.05)   — 旋转到位容差（rad）
  ~xy_goal_tolerance         (float, 默认 0.15)   — 到达目标点距离容差（m）
  ~obstacle_dist             (float, 默认 0.50)   — 前方障碍物触发距离（m）
  ~obstacle_angle            (float, 默认 0.523)  — 前方障碍检测半角（rad，≈30°）
  ~heading_correct_threshold (float, 默认 0.20)   — 前进中朝向偏差触发重新对齐（rad）
  ~set_initial_pose          (bool,  默认 False)  — 启动时是否向 AMCL 发布初始位姿
  ~initial_pose_x/y/a        (float, 默认 0.0)    — 初始位姿坐标（地图坐标系）
  ~amcl_wait_timeout         (float, 默认 30.0)   — 等待 AMCL 就绪最长时间（秒）
  ~map_rotation_offset       (float, 默认 0.0)    — 导航坐标系相对地图坐标系的
                                                     逆时针旋转角（弧度）
  ~map_translation_offset_x  (float, 默认 0.0)    — 坐标变换后叠加的 X 轴平移量（米）；
                                                     地图 origin 由 [ox_old,oy_old,θ_old]
                                                     改为 [ox_new,oy_new,θ_new] 时：
                                                     tx = ox_new - ox_old*cos(θ_new) + oy_old*sin(θ_new)
  ~map_translation_offset_y  (float, 默认 0.0)    — 坐标变换后叠加的 Y 轴平移量（米）；
                                                     ty = oy_new - ox_old*sin(θ_new) - oy_old*cos(θ_new)
"""

import math
import sys

import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ==============================================================================
#  ★ 航点配置区域 ★
#  格式与 multi_point_navigation.py 完全相同，直接复制航点列表即可。
#  'name' : 航点名称（日志显示用）
#  'x'    : 目标点在导航坐标系下的 X 坐标（米）
#  'y'    : 目标点在导航坐标系下的 Y 坐标（米）
#  'yaw'  : 到达目标点后的最终偏航角（弧度）
# ==============================================================================
WAYPOINTS = [
    {'name': '航点1',  'x':  0.174, 'y':  0.571, 'yaw':  0.935},
    {'name': '航点2',  'x':  0.717, 'y':  1.040, 'yaw':  0.729},
    {'name': '航点3',  'x':  1.037, 'y':  1.226, 'yaw':  0.532},
    {'name': '航点4',  'x':  1.708, 'y':  1.725, 'yaw':  0.515},
    {'name': '航点5',  'x':  1.404, 'y':  2.740, 'yaw':  0.729},
    {'name': '航点6',  'x':  1.794, 'y':  3.089, 'yaw':  0.784},
    {'name': '航点7',  'x':  2.419, 'y':  2.920, 'yaw':  1.121},
    {'name': '航点8',  'x':  2.833, 'y':  3.190, 'yaw':  0.482},
    {'name': '航点9',  'x':  3.129, 'y':  3.218, 'yaw': -0.530},
    {'name': '航点10', 'x':  3.540, 'y':  3.115, 'yaw': -0.743},
    {'name': '航点11', 'x':  3.540, 'y':  2.972, 'yaw': -0.781},
    {'name': '航点12', 'x':  3.838, 'y':  2.753, 'yaw': -0.751},
    {'name': '航点13', 'x':  4.090, 'y':  2.689, 'yaw': -0.999},
    {'name': '航点14', 'x':  3.817, 'y':  2.641, 'yaw': -1.639},
    {'name': '航点15', 'x':  3.443, 'y':  1.635, 'yaw': -2.254},
    {'name': '航点16', 'x':  3.276, 'y':  1.310, 'yaw':  2.654},
    {'name': '航点17', 'x':  2.462, 'y':  1.821, 'yaw':  2.640},
    {'name': '航点18', 'x':  1.880, 'y':  1.524, 'yaw': -2.324},
    {'name': '航点19', 'x':  1.079, 'y':  0.730, 'yaw': -2.562},
    {'name': '航点20', 'x': -0.265, 'y': -0.605, 'yaw':  1.081},
]
# ==============================================================================


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def _angle_diff(target, current):
    """计算两角度之差，结果规范化到 (-π, π]。"""
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _transform_waypoint(wp, rotation_offset,
                        translation_x=0.0, translation_y=0.0):
    """将航点从导航坐标系变换到地图坐标系（旋转 + 平移）。

    x_map = x_nav * cos(offset) - y_nav * sin(offset) + translation_x
    y_map = x_nav * sin(offset) + y_nav * cos(offset) + translation_y
    yaw_map = yaw_nav + offset
    """
    if rotation_offset == 0.0 and translation_x == 0.0 and translation_y == 0.0:
        return wp
    c = math.cos(rotation_offset)
    s = math.sin(rotation_offset)
    transformed = dict(wp)
    transformed['x']   = wp['x'] * c - wp['y'] * s + translation_x
    transformed['y']   = wp['x'] * s + wp['y'] * c + translation_y
    transformed['yaw'] = wp['yaw'] + rotation_offset
    return transformed


# ── 交互式终端控制（与 multi_point_navigation.py 保持一致）────────────────────

def _read_float(prompt_text, default):
    while True:
        try:
            sys.stdout.write('{} [默认 {:.4f}]: '.format(prompt_text, default))
            sys.stdout.flush()
            raw = input()
        except (EOFError, KeyboardInterrupt):
            sys.stdout.write('\n（非交互式输入，使用默认值 {:.4f}）\n'.format(default))
            sys.stdout.flush()
            return default
        raw = raw.strip()
        if raw == '':
            return default
        try:
            return float(raw)
        except ValueError:
            sys.stdout.write('  输入无效，请输入数字\n')
            sys.stdout.flush()


def interactive_prompt(next_wp, next_idx, total, label='下一个航点'):
    wp = dict(next_wp)
    sep = '─' * 60
    sys.stdout.write('\n{}\n'.format(sep))
    sys.stdout.write('【{}】  {}/{}  —  {}\n'.format(
        label, next_idx, total, wp.get('name', '未命名')))
    sys.stdout.write('  坐标：x = {:.4f}  y = {:.4f}  yaw = {:.4f} rad\n'.format(
        wp['x'], wp['y'], wp['yaw']))
    sys.stdout.write('\n请选择操作：\n')
    sys.stdout.write('  [y / 回车]  继续前往该航点\n')
    sys.stdout.write('  [m]         临时修改该航点坐标后继续\n')
    sys.stdout.write('  [n]         停止导航，退出节点\n')
    sys.stdout.write('{}\n'.format(sep))
    sys.stdout.flush()

    while True:
        try:
            sys.stdout.write('请输入选项 [y/m/n]: ')
            sys.stdout.flush()
            choice = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            sys.stdout.write('\n（非交互式输入，默认继续。）\n')
            sys.stdout.flush()
            return wp, True

        if choice in ('', 'y'):
            return wp, True
        if choice == 'n':
            sys.stdout.write('用户选择停止导航。\n')
            sys.stdout.flush()
            return wp, False
        if choice == 'm':
            sys.stdout.write('\n请输入新坐标（直接回车保留当前值）：\n')
            sys.stdout.flush()
            wp['x']   = _read_float('  新 x 坐标（米）', wp['x'])
            wp['y']   = _read_float('  新 y 坐标（米）', wp['y'])
            wp['yaw'] = _read_float('  新 yaw 角（弧度）', wp['yaw'])
            sys.stdout.write('  → 已更新：x={:.4f}  y={:.4f}  yaw={:.4f} rad\n'.format(
                wp['x'], wp['y'], wp['yaw']))
            sys.stdout.flush()
            return wp, True
        sys.stdout.write('  无效输入，请输入 y、m 或 n\n')
        sys.stdout.flush()


# ── 直线导航器 ────────────────────────────────────────────────────────────────

class StraightLineNavigator:
    """
    直线导航器：依次完成各航点的 rotate-then-translate 导航。

    导航流程（对每个航点）：
      1. 若与目标 XY 距离 < xy_goal_tolerance → 仅旋转到最终 yaw
      2. Phase 1: 原地旋转，使机器人朝向目标点方向
      3. Phase 2: 直线前进至目标点
         - 前进中若朝向偏差 > heading_correct_threshold：停下，重新对准，继续前进
         - 前进中若前方障碍物 < obstacle_dist：切换到 move_base fallback
      4. Phase 3: 原地旋转到最终 yaw
    """

    def __init__(self):
        # ── 读取参数 ──────────────────────────────────────────────────
        self.map_frame   = rospy.get_param('~map_frame',   'map')
        self.base_frame  = rospy.get_param('~base_frame',  'base_footprint')

        self.linear_speed  = rospy.get_param('~linear_speed',  0.10)   # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.30)   # rad/s
        self.yaw_tolerance = rospy.get_param('~yaw_tolerance', 0.05)   # rad
        self.xy_tolerance  = rospy.get_param('~xy_goal_tolerance', 0.15)  # m

        self.obstacle_dist  = rospy.get_param('~obstacle_dist',  0.50)  # m
        self.obstacle_angle = rospy.get_param('~obstacle_angle', 0.523) # rad (~30°)
        self.heading_correct_threshold = rospy.get_param(
            '~heading_correct_threshold', 0.20)  # rad

        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)

        self.interactive       = rospy.get_param('~interactive', True)
        self.loop              = rospy.get_param('~loop', False)
        self.loop_count        = rospy.get_param('~loop_count', 1)
        self.set_initial_pose  = rospy.get_param('~set_initial_pose', False)
        self.initial_pose_x    = rospy.get_param('~initial_pose_x', 0.0)
        self.initial_pose_y    = rospy.get_param('~initial_pose_y', 0.0)
        self.initial_pose_a    = rospy.get_param('~initial_pose_a', 0.0)
        self.amcl_wait_timeout = rospy.get_param('~amcl_wait_timeout', 30.0)
        self.rotation_offset   = rospy.get_param('~map_rotation_offset', 0.0)
        self.translation_x     = rospy.get_param('~map_translation_offset_x', 0.0)
        self.translation_y     = rospy.get_param('~map_translation_offset_y', 0.0)

        # ── 话题发布者 & TF ───────────────────────────────────────────
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()

        # ── 激光扫描订阅 ──────────────────────────────────────────────
        self._latest_scan = None
        rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=1)

        # ── move_base fallback 客户端 ─────────────────────────────────
        self._mb_client = actionlib.SimpleActionClient('move_base',
                                                        MoveBaseAction)

    # ── 话题回调 ──────────────────────────────────────────────────────────────

    def _scan_cb(self, msg):
        self._latest_scan = msg

    # ── 位姿获取 ──────────────────────────────────────────────────────────────

    def _get_pose(self):
        """从 TF 获取机器人当前位姿 (x, y, yaw)；失败返回 None。"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None

    # ── 障碍物检测 ────────────────────────────────────────────────────────────

    def _obstacle_ahead(self):
        """
        检查前方扫描锥内是否有障碍物。
        检测范围：激光扫描帧中 |beam_angle| ≤ obstacle_angle 的所有光束。
        若任一有效光束距离 < obstacle_dist 则返回 True。
        """
        scan = self._latest_scan
        if scan is None:
            return False

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min or r > scan.range_max:
                continue
            beam_angle = scan.angle_min + i * scan.angle_increment
            if abs(beam_angle) <= self.obstacle_angle:
                if r < self.obstacle_dist:
                    return True
        return False

    # ── 停止机器人 ────────────────────────────────────────────────────────────

    def _stop(self):
        """向 /cmd_vel 发布零速度，立即停止机器人。"""
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    # ── Phase 1 / 3: 原地旋转 ─────────────────────────────────────────────────

    def _rotate_to(self, target_yaw, timeout=None):
        """
        原地旋转到目标偏航角 target_yaw。
        旋转过程中 linear.x = 0，angular.z = ±angular_speed。
        返回 True 表示到位，False 表示超时或节点关闭。
        """
        if timeout is None:
            timeout = self.goal_timeout
        rate = rospy.Rate(20)
        twist = Twist()
        deadline = rospy.Time.now() + rospy.Duration(timeout)

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logwarn('原地旋转超时（%.1f s），放弃。', timeout)
                self._stop()
                return False

            pose = self._get_pose()
            if pose is None:
                rate.sleep()
                continue

            _, _, cur_yaw = pose
            error = _angle_diff(target_yaw, cur_yaw)

            if abs(error) < self.yaw_tolerance:
                self._stop()
                rospy.loginfo('✓ 旋转到位，yaw=%.4f rad（目标=%.4f rad）',
                              cur_yaw, target_yaw)
                return True

            # 仅发布角速度，线速度为 0
            twist.linear.x  = 0.0
            twist.angular.z = math.copysign(self.angular_speed, error)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._stop()
        return False

    # ── Phase 2: 直线前进 ─────────────────────────────────────────────────────

    def _drive_straight(self, goal_x, goal_y, timeout=None):
        """
        直线前进到 (goal_x, goal_y)。
        前进过程中 angular.z = 0，linear.x = linear_speed（靠近时减速）。
        若朝向偏差 > heading_correct_threshold 则停下，重新对准后继续前进。
        若前方检测到障碍物则返回 'obstacle'。
        返回值：'arrived' | 'obstacle' | 'timeout' | 'shutdown'
        """
        if timeout is None:
            timeout = self.goal_timeout
        rate = rospy.Rate(20)
        twist = Twist()
        deadline = rospy.Time.now() + rospy.Duration(timeout)

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                self._stop()
                rospy.logwarn('直线前进超时（%.1f s）。', timeout)
                return 'timeout'

            # 检查前方障碍物
            if self._obstacle_ahead():
                self._stop()
                rospy.logwarn('前方检测到障碍物（< %.2f m），切换到 move_base 绕障。',
                              self.obstacle_dist)
                return 'obstacle'

            pose = self._get_pose()
            if pose is None:
                rate.sleep()
                continue

            cur_x, cur_y, cur_yaw = pose
            dist = math.hypot(goal_x - cur_x, goal_y - cur_y)

            # 判断是否已到达目标位置
            if dist < self.xy_tolerance:
                self._stop()
                return 'arrived'

            # 检查朝向偏差（保证不同时转弯与前进）
            angle_to_goal = math.atan2(goal_y - cur_y, goal_x - cur_x)
            heading_error = _angle_diff(angle_to_goal, cur_yaw)

            if abs(heading_error) > self.heading_correct_threshold:
                # 朝向偏差超出容许范围：先停下，重新原地对准
                self._stop()
                rospy.loginfo('前进中朝向偏差 %.3f rad > %.3f rad，停止并重新对准…',
                              abs(heading_error), self.heading_correct_threshold)
                # 给重新对准保留至少 5 秒时间，避免剩余 timeout 过小导致立即超时
                _MIN_REALIGN_TIMEOUT = 5.0
                remaining = deadline.to_sec() - rospy.Time.now().to_sec()
                ok = self._rotate_to(angle_to_goal,
                                     timeout=max(_MIN_REALIGN_TIMEOUT, remaining))
                if not ok:
                    return 'timeout'
                # 对准后继续前进（重新进入循环，不发送前进指令）
                continue

            # 计算自适应速度：靠近目标时按距离比例减速，防止冲过目标。
            # SPEED_REDUCTION_FACTOR 控制减速斜率（dist*0.5 在距离约等于
            # linear_speed/0.5 米时开始低于 linear_speed）；
            # MIN_FORWARD_SPEED 防止速度降为零导致控制器停止响应。
            _SPEED_REDUCTION_FACTOR = 0.5
            _MIN_FORWARD_SPEED = 0.04  # m/s
            speed = min(self.linear_speed, dist * _SPEED_REDUCTION_FACTOR)
            speed = max(speed, _MIN_FORWARD_SPEED)

            # 只发布线速度，角速度始终为 0
            twist.linear.x  = speed
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._stop()
        return 'shutdown'

    # ── move_base fallback ────────────────────────────────────────────────────

    def _navigate_movebase(self, wp):
        """
        使用 move_base 绕障到航点 wp（地图坐标系）。
        返回 True 表示成功，False 表示失败或超时。
        """
        rospy.loginfo('使用 move_base 绕障…（目标 x=%.3f  y=%.3f）',
                      wp['x'], wp['y'])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = wp['x']
        goal.target_pose.pose.position.y = wp['y']
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, wp['yaw'])
        goal.target_pose.pose.orientation = Quaternion(*q)

        self._mb_client.send_goal(goal)
        finished = self._mb_client.wait_for_result(
            rospy.Duration(self.goal_timeout))
        if not finished:
            self._mb_client.cancel_goal()
            rospy.logwarn('move_base 绕障超时。')
            return False

        state = self._mb_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('✓ move_base 绕障成功。')
            return True

        rospy.logwarn('✗ move_base 绕障失败，状态码：%d', state)
        return False

    # ── 单个航点导航入口 ──────────────────────────────────────────────────────

    def navigate_to(self, wp):
        """
        使用 rotate-then-translate 策略导航到航点 wp（地图坐标系）。

        完整流程：
          1. 距离很近 → 仅旋转到最终 yaw
          2. Phase 1 → 旋转对准目标方向
          3. Phase 2 → 直线前进（遇障切换 move_base）
          4. Phase 3 → 旋转到最终 yaw

        返回 True 表示成功到达并调整好朝向，False 表示失败。
        """
        # 等待 TF 就绪
        rospy.sleep(0.1)
        pose = self._get_pose()
        if pose is None:
            rospy.logwarn('无法获取当前位姿（TF 未就绪），跳过该航点。')
            return False

        cur_x, cur_y, _ = pose
        goal_x  = wp['x']
        goal_y  = wp['y']
        goal_yaw = wp['yaw']
        dist = math.hypot(goal_x - cur_x, goal_y - cur_y)

        # ── 距离很近：仅旋转 ──────────────────────────────────────────
        if dist < self.xy_tolerance:
            rospy.loginfo('  与目标距离 %.3f m < %.3f m，仅原地旋转到最终 yaw。',
                          dist, self.xy_tolerance)
            return self._rotate_to(goal_yaw)

        # ── Phase 1: 旋转对准目标方向 ─────────────────────────────────
        angle_to_goal = math.atan2(goal_y - cur_y, goal_x - cur_x)
        rospy.loginfo('  [Phase 1] 旋转对准目标方向 %.4f rad…', angle_to_goal)
        ok = self._rotate_to(angle_to_goal)
        if not ok:
            rospy.logwarn('  Phase 1 旋转失败，切换到 move_base 完成该航点。')
            return self._navigate_movebase(wp)

        # ── Phase 2: 直线前进 ─────────────────────────────────────────
        rospy.loginfo('  [Phase 2] 直线前进至目标（距离 %.3f m）…', dist)
        result = self._drive_straight(goal_x, goal_y)

        if result == 'obstacle':
            # 使用 move_base 绕障到达完整目标（含最终 yaw）
            success = self._navigate_movebase(wp)
            # move_base 已经处理了最终朝向，直接返回
            return success

        if result not in ('arrived',):
            rospy.logwarn('  Phase 2 前进失败（%s），切换到 move_base。', result)
            return self._navigate_movebase(wp)

        # ── Phase 3: 旋转到最终 yaw ──────────────────────────────────
        rospy.loginfo('  [Phase 3] 旋转到最终朝向 %.4f rad…', goal_yaw)
        self._rotate_to(goal_yaw)  # 最终旋转失败不视为整体失败

        return True

    # ── AMCL 等待 & 初始位姿 ─────────────────────────────────────────────────

    def _publish_initial_pose(self):
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        rospy.sleep(1.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp    = rospy.Time.now()
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        q = quaternion_from_euler(0.0, 0.0, self.initial_pose_a)
        msg.pose.pose.orientation = Quaternion(*q)
        cov = list(msg.pose.covariance)
        cov[0]  = 0.25    # x 方差（m²），covariance 矩阵 [0,0]
        cov[7]  = 0.25    # y 方差（m²），covariance 矩阵 [1,1]
        cov[35] = 0.0685  # yaw 方差（rad²，约 ±15°），covariance 矩阵 [5,5]
        msg.pose.covariance = cov
        pub.publish(msg)
        rospy.loginfo('已向 AMCL 发布初始位姿：x=%.4f  y=%.4f  yaw=%.4f rad',
                      self.initial_pose_x, self.initial_pose_y,
                      self.initial_pose_a)
        rospy.sleep(1.0)

    def _wait_for_amcl(self):
        if self.amcl_wait_timeout <= 0.0:
            rospy.loginfo('amcl_wait_timeout≤0，跳过 AMCL 等待。')
            return
        rospy.loginfo('等待 AMCL 定位就绪（最多 %.0f 秒）…',
                      self.amcl_wait_timeout)
        try:
            rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                                   timeout=self.amcl_wait_timeout)
            rospy.loginfo('✓ AMCL 已就绪。')
        except rospy.ROSException:
            rospy.logwarn('等待 AMCL 超时，导航精度可能受影响。')

    # ── 主导航循环 ────────────────────────────────────────────────────────────

    def run(self):
        """执行完整的多点导航任务（含循环模式）。"""
        if not WAYPOINTS:
            rospy.logerr('航点列表为空，请在脚本顶部 WAYPOINTS 中添加导航点。')
            return

        # ── 初始化：发布初始位姿、等待 AMCL ──────────────────────────
        if self.set_initial_pose:
            self._publish_initial_pose()
        self._wait_for_amcl()

        # ── 等待 move_base 就绪（fallback 需要）─────────────────────
        rospy.loginfo('正在连接 move_base 动作服务器…')
        if not self._mb_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr('无法连接 move_base，请确认 navigation 已启动。')
            return
        rospy.loginfo('✓ 已连接 move_base。')

        # ── 等待激光扫描数据就绪 ─────────────────────────────────────
        rospy.loginfo('等待激光扫描数据就绪…')
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while self._latest_scan is None and not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logwarn('等待 /scan 超时，障碍物检测可能不可用。')
                break
            rospy.sleep(0.2)
        if self._latest_scan is not None:
            rospy.loginfo('✓ 激光扫描已就绪。')

        total = len(WAYPOINTS)
        rospy.loginfo('=' * 50)
        rospy.loginfo('直线导航节点已启动')
        rospy.loginfo('  航点数量    : %d', total)
        rospy.loginfo('  循环模式    : %s', '开启' if self.loop else '关闭')
        rospy.loginfo('  线速度      : %.2f m/s', self.linear_speed)
        rospy.loginfo('  角速度      : %.2f rad/s', self.angular_speed)
        rospy.loginfo('  障碍距离阈值: %.2f m', self.obstacle_dist)
        rospy.loginfo('=' * 50)

        # 复制航点列表（交互模式可临时修改）
        pending = [dict(wp) for wp in WAYPOINTS]

        # 交互模式：执行第一个航点前询问用户
        if self.interactive and total > 0:
            first_wp, ok = interactive_prompt(pending[0], 1, total,
                                              label='起始航点')
            if not ok:
                rospy.loginfo('用户选择停止，导航未启动。')
                return
            pending[0] = first_wp

        run = 0
        while not rospy.is_shutdown():
            run += 1
            rospy.loginfo('━' * 50)
            if self.loop:
                rospy.loginfo('开始第 %d 轮巡逻（共 %s 轮）',
                              run, '无限' if self.loop_count == 0
                              else str(self.loop_count))
            else:
                rospy.loginfo('开始单次多点导航任务')

            success = self._run_once(pending)

            if not success:
                rospy.logwarn('本轮导航未能完成所有航点，节点退出。')
                break

            rospy.loginfo('✓ 已完成所有 %d 个航点。', total)

            if not self.loop:
                rospy.loginfo('非循环模式，导航任务完成，节点退出。')
                break
            if self.loop_count != 0 and run >= self.loop_count:
                rospy.loginfo('已完成 %d 轮巡逻，节点退出。', run)
                break

            rospy.loginfo('循环模式：即将开始下一轮…')

    def _run_once(self, pending):
        """执行一轮航点导航，返回 True 表示全部完成。"""
        total = len(pending)
        for idx, wp in enumerate(pending):
            if rospy.is_shutdown():
                return False

            name = wp.get('name', '未命名')
            rospy.loginfo('─' * 50)
            rospy.loginfo('正在前往第 %d/%d 个航点：%s', idx + 1, total, name)
            rospy.loginfo('  坐标（导航系）：x=%.4f  y=%.4f  yaw=%.4f rad',
                          wp['x'], wp['y'], wp['yaw'])

            # 坐标变换到地图坐标系
            wp_map = _transform_waypoint(wp, self.rotation_offset,
                                         self.translation_x, self.translation_y)
            if self.rotation_offset != 0.0 or self.translation_x != 0.0 or self.translation_y != 0.0:
                rospy.loginfo('  坐标（地图系）：x=%.4f  y=%.4f  yaw=%.4f rad',
                              wp_map['x'], wp_map['y'], wp_map['yaw'])

            # 执行导航
            ok = self.navigate_to(wp_map)
            if not ok:
                rospy.logwarn('✗ 未能到达航点 [%s]，导航中止。', name)
                return False

            rospy.loginfo('✓ 成功到达航点 [%s]', name)

            # 交互模式：询问下一步
            next_idx = idx + 1
            if next_idx < total and self.interactive:
                next_wp, should_continue = interactive_prompt(
                    pending[next_idx], next_idx + 1, total)
                if not should_continue:
                    rospy.loginfo('用户选择停止，导航提前结束。')
                    return False
                pending[next_idx] = next_wp

        return True


# ── 节点入口 ──────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('straight_line_nav', anonymous=False)
    nav = StraightLineNavigator()
    nav.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
