#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多点导航节点（交互式终端控制版）
================================
本节点基于 ROS move_base 动作服务器，实现机器人按照预设的航点列表依次导航。
每到达一个航点后，终端会显示下一个航点的坐标，并提示用户选择：
  继续 / 停止 / 临时修改下一个航点的坐标后继续。

使用说明：
  1. 确保 navigation.launch 已启动（底盘、地图服务器、AMCL、move_base）。
  2. 通过 multi_point_navigation.launch 启动本节点（推荐），或直接运行：
       rosrun wulina multi_point_navigation.py
  3. 启动后按终端提示操作即可。

经测试反馈的问题说明与解决方案
--------------------------------
【问题1】同位置旋转时产生 XY 偏移
  原因：move_base 在处理与当前位置 XY 几乎相同的目标时，为同时满足位置
  容差和角度容差，会让底盘做微小移动，导致轮式里程计和 AMCL 位姿出现偏差。
  解决方案：当相邻两个航点的 XY 距离小于 rotate_in_place_threshold（默认
  0.05 m）时，本节点绕过 move_base，直接向 /cmd_vel 发布角速度指令来原地
  旋转，旋转完成后立即停止，从而消除 move_base 引入的额外位移。

【问题2】AMCL 初始位姿与建图起点不一致
  原因：AMCL 默认将机器人初始位姿估计为地图坐标原点 (x=0, y=0, yaw=0)，
  这正是建图时（运行 gmapping/cartographer）机器人所在的位置和朝向。
  如果机器人每次启动导航时所处的物理位置和朝向与建图起点完全一致，则
  无需任何额外设置；否则，AMCL 需要一段时间（粒子滤波收敛）才能正确
  定位，期间导航目标可能偏离。
  解决方案：
    a) 每次导航前，在 RViz 中使用"2D Pose Estimate"工具手动标注当前位置。
    b) 若机器人每次都放回固定的非原点位置，可将 set_initial_pose 设为 true
       并填写 initial_pose_x/y/a 参数，节点启动时会自动向 AMCL 发布初始
       位姿，省去手动标注步骤。

【问题3】改变 XY（含或不含角度）时导航终点偏差大、路径奇怪
  原因：
    a) AMCL 尚未收敛。节点启动后 move_base 立即开始规划，若 AMCL 粒子
       滤波此时还没有读到足够的激光数据完成收敛，机器人对自身在地图中的
       位置估计是错误的，导致规划路径的起点偏离实际位置，最终停靠点也
       随之偏移，路径也会显得"绕路"或"走错方向"。
    b) prev_x/prev_y 使用目标坐标而非实际位置。每到达一个航点后，代码以
       航点的目标坐标作为下一次距离比较的"上一位置"，若小车因里程计误差
       实际停在略微不同的位置，累积偏差会影响后续旋转/移动决策。
  解决方案：
    a) 导航前等待 AMCL 就绪：新增 _wait_for_amcl() 函数，在节点启动后
       以及发布初始位姿后，等待 /amcl_pose 话题发布至少一帧消息，确认
       AMCL 粒子滤波已激活并完成初步定位后再开始导航。
    b) 用 TF 实际位置更新 prev_x/prev_y：每个航点完成后，通过 TF 读取
       机器人的真实位置（map→base_footprint），若 TF 查询失败则退化为
       使用目标坐标，保证旋转阈值判断的准确性。

【问题4】地图存在角度偏差，需要导航坐标系相对建图坐标系旋转
  原因：建图时机器人的初始朝向（即地图坐标系的 X 轴方向）与期望的导航
  坐标系 X 轴方向存在固定角度偏差。若直接使用地图坐标系的坐标填写航点，
  所有目标点的方向都会产生系统性偏差。
  解决方案：新增 map_rotation_offset 参数（弧度），表示导航（用户）坐标系
  相对地图坐标系的逆时针旋转角度。节点在将航点发送给 move_base 前，
  自动完成坐标变换：
    x_map = x_nav * cos(offset) - y_nav * sin(offset)
    y_map = x_nav * sin(offset) + y_nav * cos(offset)
    yaw_map = yaw_nav + offset
  这样用户只需在自己习惯的坐标系中填写航点，节点负责转换为地图坐标系。

参数说明（可通过 ROS 参数服务器或 launch 文件覆盖）：
  ~loop                      (bool,  默认: False) — 是否循环执行航点列表
  ~loop_count                (int,   默认: 1)     — 循环次数（loop=False 时忽略）；0=无限
  ~goal_timeout              (float, 默认: 60.0)  — 单个目标的超时时间（秒）
  ~map_frame                 (str,   默认: "map") — 导航坐标系
  ~base_frame                (str,   默认: "base_footprint") — 机器人本体坐标系
  ~interactive               (bool,  默认: True)  — 是否在每步之间显示终端交互提示
  ~rotate_in_place_threshold (float, 默认: 0.05)  — XY 距离（米）小于此值时改用原地旋转
  ~angular_speed             (float, 默认: 0.3)   — 原地旋转角速度（rad/s）
  ~yaw_tolerance             (float, 默认: 0.05)  — 原地旋转角度容差（rad）
  ~set_initial_pose          (bool,  默认: False) — 启动时是否向 AMCL 发布初始位姿
  ~initial_pose_x            (float, 默认: 0.0)   — 初始位姿 X（米，地图坐标系）
  ~initial_pose_y            (float, 默认: 0.0)   — 初始位姿 Y（米，地图坐标系）
  ~initial_pose_a            (float, 默认: 0.0)   — 初始位姿偏航角（弧度，地图坐标系）
  ~amcl_wait_timeout         (float, 默认: 30.0)  — 等待 AMCL 就绪的最长时间（秒）；
                                                     设为 ≤0 跳过等待
  ~map_rotation_offset       (float, 默认: 0.0)   — 导航坐标系相对地图坐标系的逆时针
                                                     旋转角（弧度）；例如 π/4 ≈ 0.7854
                                                     表示导航坐标系比地图坐标系偏转 45°
"""

import math
import sys
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ==============================================================================
#  ★ 航点配置区域 ★
#  格式说明：
#    'name' : 航点名称（仅用于日志显示）
#    'x'    : 目标点在导航坐标系下的 X 坐标（米）
#    'y'    : 目标点在导航坐标系下的 Y 坐标（米）
#    'yaw'  : 到达目标点后的偏航角（弧度），相对于导航坐标系
#             正值 = 逆时针，负值 = 顺时针；0 = 导航坐标系 X 轴正方向
#
#  导航坐标系说明：
#    当 map_rotation_offset = 0（默认）时，导航坐标系 = 地图坐标系（map frame），
#    坐标直接来自 RViz 2D Nav Goal 工具。
#    当 map_rotation_offset ≠ 0 时，导航坐标系相对地图坐标系逆时针旋转该角度，
#    节点启动时会自动将坐标转换到地图坐标系后再发送给 move_base。
#
#  以下坐标由 RViz 2D Nav Goal 工具采集，已换算为 yaw 角。
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


def build_goal(waypoint, map_frame):
    """
    根据航点字典构造 MoveBaseGoal 消息。

    参数:
        waypoint  (dict): 包含 'x'、'y'、'yaw' 键的航点字典
        map_frame (str):  目标坐标系名称（通常为 "map"）

    返回:
        MoveBaseGoal: 填充好的目标消息
    """
    goal = MoveBaseGoal()

    # 设置坐标系与时间戳
    goal.target_pose.header.frame_id = map_frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置（X、Y；Z 固定为 0，平面导航不需要高度）
    goal.target_pose.pose.position.x = waypoint['x']
    goal.target_pose.pose.position.y = waypoint['y']
    goal.target_pose.pose.position.z = 0.0

    # 将偏航角（欧拉角）转换为四元数并赋值给目标朝向
    q = quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
    goal.target_pose.pose.orientation = Quaternion(*q)

    return goal


def _transform_waypoint(wp, rotation_offset):
    """
    将航点坐标从导航（用户）坐标系变换到地图坐标系（map frame）。

    当导航坐标系相对建图时的 map 坐标系存在逆时针旋转偏差时，必须在把
    目标发送给 move_base 前完成坐标变换，否则实际导航目标会出现方向性偏差。

    变换原理（导航坐标系 → 地图坐标系）：
      设 offset = 导航坐标系相对地图坐标系的逆时针旋转角（rad），
      即导航坐标系的 X 轴在地图坐标系中的方向为 (cos offset, sin offset)，
      则点 (x_nav, y_nav) 在地图坐标系中的坐标为：

        x_map = x_nav * cos(offset) - y_nav * sin(offset)
        y_map = x_nav * sin(offset) + y_nav * cos(offset)
        yaw_map = yaw_nav + offset

    参数:
        wp              (dict):  原始航点字典（含 'x'、'y'、'yaw' 及其他键）
        rotation_offset (float): 导航坐标系相对地图坐标系的逆时针偏角（弧度）

    返回:
        dict: 地图坐标系下的航点字典（非坐标键原样保留）
    """
    if rotation_offset == 0.0:
        return wp  # 无旋转偏差，无需变换

    c = math.cos(rotation_offset)
    s = math.sin(rotation_offset)
    transformed = dict(wp)
    transformed['x']   = wp['x'] * c - wp['y'] * s
    transformed['y']   = wp['x'] * s + wp['y'] * c
    transformed['yaw'] = wp['yaw'] + rotation_offset
    return transformed


# ------------------------------------------------------------------------------
#  原地旋转辅助函数（用于解决同位置纯角度变化时的 XY 漂移问题）
# ------------------------------------------------------------------------------

def _angle_diff(target, current):
    """
    计算目标角与当前角之差，结果规范化到 (-π, π] 区间。

    参数:
        target  (float): 目标偏航角（弧度）
        current (float): 当前偏航角（弧度）

    返回:
        float: 角度差（弧度），正值表示需逆时针旋转，负值表示需顺时针旋转
    """
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _rotate_in_place(target_yaw, cmd_vel_pub, tf_listener,
                     map_frame, base_frame,
                     angular_speed=0.3, yaw_tolerance=0.05, timeout=15.0):
    """
    直接向 /cmd_vel 发布角速度指令，原地旋转到目标 yaw 角。
    旋转到位后立即停止，不依赖 move_base，从而避免位置修正引入的 XY 偏移。

    参数:
        target_yaw    (float):              目标偏航角（弧度）
        cmd_vel_pub   (rospy.Publisher):    /cmd_vel 话题发布者
        tf_listener   (tf.TransformListener): TF 监听器
        map_frame     (str):                地图坐标系名称
        base_frame    (str):                机器人本体坐标系名称
        angular_speed (float):              旋转角速度（rad/s），正值有效
        yaw_tolerance (float):              到位判据（rad），误差小于此值即停止
        timeout       (float):              最长等待时间（秒）

    返回:
        bool: 在超时时间内成功到达目标角度返回 True，否则返回 False
    """
    rate = rospy.Rate(20)
    twist = Twist()
    deadline = rospy.Time.now() + rospy.Duration(timeout)

    while not rospy.is_shutdown():
        if rospy.Time.now() > deadline:
            rospy.logwarn('原地旋转超时（%.1f 秒），放弃。', timeout)
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            return False

        try:
            (_, rot) = tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rate.sleep()
            continue

        _, _, current_yaw = euler_from_quaternion(rot)
        error = _angle_diff(target_yaw, current_yaw)

        if abs(error) < yaw_tolerance:
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            rospy.loginfo('✓ 原地旋转完成，当前 yaw=%.4f rad（目标=%.4f rad）',
                          current_yaw, target_yaw)
            return True

        twist.angular.z = math.copysign(angular_speed, error)
        cmd_vel_pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    return False


def _publish_initial_pose(pub, x, y, yaw, map_frame):
    """
    向 /initialpose 话题发布位姿估计，告知 AMCL 机器人当前所在位置。

    参数:
        pub       (rospy.Publisher): /initialpose 话题发布者
        x         (float):           初始位姿 X（米）
        y         (float):           初始位姿 Y（米）
        yaw       (float):           初始偏航角（弧度）
        map_frame (str):             地图坐标系名称
    """
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = map_frame
    pose_msg.header.stamp = rospy.Time.now()

    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    pose_msg.pose.pose.position.z = 0.0

    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose_msg.pose.pose.orientation = Quaternion(*q)

    # 设置位置和角度的方差（与 RViz "2D Pose Estimate" 的默认值一致）
    cov = list(pose_msg.pose.covariance)
    cov[0]  = 0.25    # x 方差（m²）
    cov[7]  = 0.25    # y 方差（m²）
    cov[35] = 0.0685  # yaw 方差（rad²，约 ±15°）
    pose_msg.pose.covariance = cov

    pub.publish(pose_msg)
    rospy.loginfo('已向 AMCL 发布初始位姿：x=%.4f  y=%.4f  yaw=%.4f rad',
                  x, y, yaw)


def _wait_for_amcl(timeout=30.0):
    """
    等待 AMCL 发布第一个位姿估计消息，确保定位系统已就绪后再开始导航。

    AMCL（自适应蒙特卡洛定位）在启动后需要读取初始的激光扫描数据并运行
    粒子滤波，才能在地图中确定机器人的位置。若在 AMCL 收敛前就向 move_base
    发送导航目标，规划器将从错误的起始位置出发，导致路径奇怪和终点偏差大。

    本函数通过订阅 /amcl_pose 话题，等待 AMCL 发布至少一帧位姿消息。
    这是最低限度的"AMCL 已激活"判据；若需要更高精度，请在 RViz 中目视
    确认粒子云已收敛（粒子分布收紧到一个小区域）后再允许节点继续。

    参数:
        timeout (float): 最长等待时间（秒）；若 ≤ 0 则跳过等待直接返回 True

    返回:
        bool: AMCL 在超时时间内发布了位姿则返回 True，否则返回 False
    """
    if timeout <= 0.0:
        rospy.loginfo('amcl_wait_timeout≤0，跳过 AMCL 就绪等待。')
        return True

    rospy.loginfo('正在等待 AMCL 定位就绪（最多 %.0f 秒）…', timeout)
    try:
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped,
                               timeout=timeout)
        rospy.loginfo('✓ AMCL 定位已就绪，可以开始导航。')
        return True
    except rospy.ROSException:
        rospy.logwarn(
            '等待 AMCL 超时（%.0f 秒）。定位可能尚未收敛，导航精度可能受影响。\n'
            '  建议：在 RViz 中使用"2D Pose Estimate"手动指定初始位姿，\n'
            '  或将 set_initial_pose 设为 true 并配置 initial_pose_x/y/a。',
            timeout)
        return False


# ------------------------------------------------------------------------------
#  交互式终端控制
# ------------------------------------------------------------------------------

def _read_float(prompt_text, default):
    """
    在终端打印提示并读取一个浮点数。
    若用户直接回车，则使用默认值。
    若终端不可交互（EOFError），返回默认值并打印提示。

    参数:
        prompt_text (str):   显示给用户的提示文字
        default     (float): 默认值

    返回:
        float: 用户输入的值或默认值
    """
    while True:
        try:
            sys.stdout.write('{} [默认 {:.4f}]: '.format(prompt_text, default))
            sys.stdout.flush()
            raw = input()
        except (EOFError, KeyboardInterrupt):
            sys.stdout.write('\n（检测到非交互式输入或中断，使用默认值 {:.4f}）\n'.format(default))
            sys.stdout.flush()
            return default

        raw = raw.strip()
        if raw == '':
            return default
        try:
            return float(raw)
        except ValueError:
            sys.stdout.write('  输入无效，请输入一个数字（如 1.23 或 -0.5）\n')
            sys.stdout.flush()


def interactive_prompt(next_wp, next_idx, total, label='下一个航点'):
    """
    在终端显示指定航点的信息并询问用户如何操作。

    参数:
        next_wp   (dict): 航点字典（含 'name'、'x'、'y'、'yaw'）
        next_idx  (int):  航点在列表中的序号（1-based）
        total     (int):  航点总数
        label     (str):  显示在标题行的标签，如"起始航点"或"下一个航点"

    返回:
        tuple(dict, bool):
            - 实际使用的航点字典（可能被用户临时修改）
            - True = 继续导航，False = 停止导航
    """
    wp = dict(next_wp)  # 浅拷贝，避免修改原始列表

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
            sys.stdout.write('\n（非交互式输入或中断，默认继续。）\n')
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
            wp['x'] = _read_float('  新 x 坐标（米）', wp['x'])
            wp['y'] = _read_float('  新 y 坐标（米）', wp['y'])
            wp['yaw'] = _read_float('  新 yaw 角（弧度）', wp['yaw'])
            sys.stdout.write('  → 已更新：x={:.4f}  y={:.4f}  yaw={:.4f} rad\n'.format(
                wp['x'], wp['y'], wp['yaw']))
            sys.stdout.flush()
            return wp, True

        sys.stdout.write('  无效输入，请输入 y、m 或 n\n')
        sys.stdout.flush()


# ------------------------------------------------------------------------------
#  导航执行
# ------------------------------------------------------------------------------

def navigate_to_waypoints(client, waypoints, map_frame, goal_timeout,
                           interactive=True,
                           cmd_vel_pub=None, tf_listener=None,
                           rotate_in_place_threshold=0.05,
                           base_frame='base_footprint',
                           angular_speed=0.3, yaw_tolerance=0.05,
                           initial_pose_x=0.0, initial_pose_y=0.0,
                           rotation_offset=0.0):
    """
    按顺序导航到航点列表中的每个目标点。
    当 interactive=True 时：
      - 执行第一个航点前，先在终端显示起始航点坐标并等待用户确认/修改/停止。
      - 每到达一个航点（最后一个除外），在终端显示下一个航点信息并等待用户指令。

    当相邻两个航点的 XY 距离小于 rotate_in_place_threshold 时，
    使用原地旋转代替 move_base，避免位置修正引入的 XY 漂移。

    当 rotation_offset ≠ 0.0 时，所有航点坐标在发送给 move_base 之前会先
    通过 _transform_waypoint() 从导航坐标系变换到地图坐标系。

    参数:
        client                   (SimpleActionClient): move_base 动作客户端
        waypoints                (list):               航点字典列表（导航坐标系）
        map_frame                (str):                目标坐标系名称
        goal_timeout             (float):              单个目标的超时时间（秒）
        interactive              (bool):               是否启用交互式终端控制
        cmd_vel_pub              (rospy.Publisher|None): /cmd_vel 发布者，原地旋转时使用
        tf_listener              (tf.TransformListener|None): TF 监听器，原地旋转时使用
        rotate_in_place_threshold (float):             XY 距离阈值（米），小于此值则原地旋转
        base_frame               (str):                机器人本体坐标系（原地旋转时使用）
        angular_speed            (float):              原地旋转角速度（rad/s）
        yaw_tolerance            (float):              原地旋转角度容差（rad）
        initial_pose_x           (float):              初始位置 X（米，地图坐标系），用于首航点距离判断
        initial_pose_y           (float):              初始位置 Y（米，地图坐标系）
        rotation_offset          (float):              导航坐标系相对地图坐标系的逆时针偏角（rad）

    返回:
        bool: 所有航点均成功到达时返回 True，否则返回 False
    """
    total = len(waypoints)

    # 将原始列表复制一份，以便交互模式下可对航点临时修改而不影响原列表
    pending = [dict(wp) for wp in waypoints]
    idx = 0

    # 跟踪上一个已到达位置，用于判断是否需要原地旋转。
    # 初始值为机器人的初始位姿（或地图原点）。
    prev_x, prev_y = initial_pose_x, initial_pose_y

    # 交互模式：在执行第一个航点前先询问用户
    if interactive and total > 0:
        first_wp, should_continue = interactive_prompt(
            pending[0], 1, total, label='起始航点')
        if not should_continue:
            rospy.loginfo('用户选择停止，导航未启动。')
            return False
        pending[0] = first_wp

    while idx < total:
        # 检查节点是否已被关闭
        if rospy.is_shutdown():
            rospy.logwarn('节点已关闭，停止导航。')
            return False

        wp = pending[idx]
        name = wp.get('name', '未命名航点')
        rospy.loginfo('─' * 50)
        rospy.loginfo('正在前往第 %d/%d 个航点：%s', idx + 1, total, name)
        rospy.loginfo('  目标坐标（导航坐标系）：x=%.4f  y=%.4f  yaw=%.4f rad',
                      wp['x'], wp['y'], wp['yaw'])

        # 将航点从导航坐标系变换到地图坐标系（move_base 使用地图坐标系）
        wp_map = _transform_waypoint(wp, rotation_offset)
        if rotation_offset != 0.0:
            rospy.loginfo('  目标坐标（地图坐标系）：x=%.4f  y=%.4f  yaw=%.4f rad',
                          wp_map['x'], wp_map['y'], wp_map['yaw'])

        # 计算本航点（地图坐标系）与上一位置（地图坐标系）的 XY 距离
        xy_dist = math.hypot(wp_map['x'] - prev_x, wp_map['y'] - prev_y)

        # 判断是否使用原地旋转（XY 几乎不变，仅角度变化）
        use_rotate = (xy_dist < rotate_in_place_threshold
                      and cmd_vel_pub is not None
                      and tf_listener is not None)

        if use_rotate:
            rospy.loginfo('  XY 距离 %.4f m < 阈值 %.4f m，使用原地旋转代替 move_base',
                          xy_dist, rotate_in_place_threshold)
            ok = _rotate_in_place(
                wp_map['yaw'], cmd_vel_pub, tf_listener,
                map_frame, base_frame,
                angular_speed=angular_speed,
                yaw_tolerance=yaw_tolerance,
                timeout=goal_timeout)
            if not ok:
                rospy.logwarn('✗ 航点 [%s] 原地旋转失败或超时。', name)
                return False
        else:
            # 构造并发送 move_base 目标（使用地图坐标系下的航点）
            goal = build_goal(wp_map, map_frame)
            client.send_goal(goal)

            # 等待结果，设置超时
            finished = client.wait_for_result(rospy.Duration(goal_timeout))

            if not finished:
                rospy.logwarn('航点 [%s] 导航超时（%.1f 秒），取消当前目标。',
                              name, goal_timeout)
                client.cancel_goal()
                return False

            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('✓ 成功到达航点 [%s]', name)
            else:
                rospy.logwarn('✗ 未能到达航点 [%s]，状态码：%d', name, state)
                return False

        # 更新上一位置：优先从 TF 读取实际位置，确保旋转阈值判断准确。
        # 若 TF 查询失败（定位丢失等异常情况），退化为使用地图坐标系目标坐标。
        if tf_listener is not None:
            try:
                (trans, _) = tf_listener.lookupTransform(
                    map_frame, base_frame, rospy.Time(0))
                prev_x, prev_y = trans[0], trans[1]
                rospy.logdebug('实际到达位置（TF）：x=%.4f  y=%.4f', prev_x, prev_y)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                rospy.logwarn('TF 查询失败，使用航点目标坐标（地图坐标系）作为上一位置。')
                prev_x, prev_y = wp_map['x'], wp_map['y']
        else:
            prev_x, prev_y = wp_map['x'], wp_map['y']

        # 判断是否还有下一个航点，以及是否需要交互提示
        next_idx = idx + 1
        if next_idx < total and interactive:
            next_wp, should_continue = interactive_prompt(
                pending[next_idx], next_idx + 1, total)
            if not should_continue:
                rospy.loginfo('用户选择停止，导航提前结束。')
                return False
            # 将（可能被修改的）下一个航点写回待执行列表
            pending[next_idx] = next_wp

        idx += 1

    return True


def main():
    """
    多点导航主函数。
    初始化 ROS 节点，连接 move_base 动作服务器，然后按配置执行导航循环。
    """
    # 初始化 ROS 节点
    rospy.init_node('multi_point_navigation', anonymous=False)

    # ------------------------------------------------------------------
    # 从参数服务器读取运行参数（可在 launch 文件或命令行中覆盖）
    # ------------------------------------------------------------------
    loop         = rospy.get_param('~loop', False)
    loop_count   = rospy.get_param('~loop_count', 1)
    goal_timeout = rospy.get_param('~goal_timeout', 60.0)
    map_frame    = rospy.get_param('~map_frame', 'map')
    base_frame   = rospy.get_param('~base_frame', 'base_footprint')
    # 是否启用交互式终端控制（每步之间在终端提示用户）
    interactive  = rospy.get_param('~interactive', True)

    # 原地旋转相关参数（解决同位置旋转时 XY 漂移问题）
    rotate_threshold = rospy.get_param('~rotate_in_place_threshold', 0.05)
    angular_speed    = rospy.get_param('~angular_speed', 0.3)
    yaw_tolerance    = rospy.get_param('~yaw_tolerance', 0.05)

    # 初始位姿相关参数（解决 AMCL 初始位姿与建图起点不一致问题）
    set_initial_pose  = rospy.get_param('~set_initial_pose', False)
    initial_pose_x    = rospy.get_param('~initial_pose_x', 0.0)
    initial_pose_y    = rospy.get_param('~initial_pose_y', 0.0)
    initial_pose_a    = rospy.get_param('~initial_pose_a', 0.0)

    # 等待 AMCL 就绪的超时时间（解决导航终点偏差大/路径奇怪问题）
    amcl_wait_timeout = rospy.get_param('~amcl_wait_timeout', 30.0)

    # 坐标系旋转偏差（解决地图角度偏差问题）
    # 表示导航（用户）坐标系相对地图坐标系的逆时针旋转角（弧度）
    # 例：地图偏差 45°（π/4 ≈ 0.7854），则设为 0.7854
    map_rotation_offset = rospy.get_param('~map_rotation_offset', 0.0)

    rospy.loginfo('=' * 50)
    rospy.loginfo('多点导航节点已启动')
    rospy.loginfo('  航点数量    : %d', len(WAYPOINTS))
    rospy.loginfo('  循环模式    : %s', '开启' if loop else '关闭')
    if loop:
        rospy.loginfo('  循环次数    : %s', '无限' if loop_count == 0 else str(loop_count))
    rospy.loginfo('  目标超时    : %.1f 秒', goal_timeout)
    rospy.loginfo('  坐标系      : %s', map_frame)
    rospy.loginfo('  本体坐标系  : %s', base_frame)
    rospy.loginfo('  交互控制    : %s', '开启' if interactive else '关闭')
    rospy.loginfo('  原地旋转阈值: %.3f m', rotate_threshold)
    rospy.loginfo('  AMCL 等待   : %.0f 秒', amcl_wait_timeout)
    if map_rotation_offset != 0.0:
        rospy.loginfo('  坐标旋转偏差: %.4f rad（%.2f°，导航坐标系相对地图逆时针偏转）',
                      map_rotation_offset, math.degrees(map_rotation_offset))
    if set_initial_pose:
        rospy.loginfo('  初始位姿    : x=%.4f  y=%.4f  yaw=%.4f rad',
                      initial_pose_x, initial_pose_y, initial_pose_a)
    rospy.loginfo('=' * 50)

    # ------------------------------------------------------------------
    # 检查航点列表是否为空
    # ------------------------------------------------------------------
    if not WAYPOINTS:
        rospy.logerr('航点列表为空，请在脚本顶部的 WAYPOINTS 列表中添加导航点。')
        return

    # ------------------------------------------------------------------
    # 创建话题发布者和 TF 监听器（用于原地旋转功能）
    # ------------------------------------------------------------------
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    tf_listener = tf.TransformListener()

    # ------------------------------------------------------------------
    # 向 AMCL 发布初始位姿估计（解决建图起点与导航起点不一致的问题）
    # ------------------------------------------------------------------
    if set_initial_pose:
        init_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        # 等待订阅者就绪（AMCL 订阅 /initialpose 需要短暂延迟）
        rospy.sleep(1.0)
        _publish_initial_pose(init_pose_pub, initial_pose_x, initial_pose_y,
                              initial_pose_a, map_frame)
        # 给 AMCL 粒子滤波一点时间重新收敛
        rospy.sleep(1.0)

    # ------------------------------------------------------------------
    # 等待 AMCL 就绪（解决导航路径奇怪和终点偏差大的问题）
    # 必须在 AMCL 完成初步定位后再启动导航，否则 move_base 会从错误的
    # 起始位置进行规划，导致路径偏离和终点不准确。
    # ------------------------------------------------------------------
    _wait_for_amcl(timeout=amcl_wait_timeout)

    # ------------------------------------------------------------------
    # 创建 move_base 动作客户端并等待服务器就绪
    # ------------------------------------------------------------------
    rospy.loginfo('正在连接 move_base 动作服务器...')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 最多等待 30 秒，若 move_base 未启动则退出
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr('无法连接到 move_base 动作服务器，请确认 navigation.launch 已启动。')
        return

    rospy.loginfo('已成功连接到 move_base 动作服务器。')

    # ------------------------------------------------------------------
    # 导航循环
    # ------------------------------------------------------------------
    run = 0
    success = True

    while not rospy.is_shutdown():
        run += 1
        rospy.loginfo('━' * 50)
        if loop:
            rospy.loginfo('开始第 %d 轮巡逻（共 %s 轮）',
                          run, '无限' if loop_count == 0 else str(loop_count))
        else:
            rospy.loginfo('开始单次多点导航任务')

        success = navigate_to_waypoints(
            client, WAYPOINTS, map_frame, goal_timeout, interactive,
            cmd_vel_pub=cmd_vel_pub,
            tf_listener=tf_listener,
            rotate_in_place_threshold=rotate_threshold,
            base_frame=base_frame,
            angular_speed=angular_speed,
            yaw_tolerance=yaw_tolerance,
            initial_pose_x=initial_pose_x,
            initial_pose_y=initial_pose_y,
            rotation_offset=map_rotation_offset)

        if not success:
            rospy.logwarn('本轮导航未能完成所有航点，节点退出。')
            break

        rospy.loginfo('━' * 50)
        rospy.loginfo('✓ 已完成所有 %d 个航点的导航。', len(WAYPOINTS))

        if not loop:
            rospy.loginfo('非循环模式，导航任务已全部完成。节点退出。')
            break

        if loop_count != 0 and run >= loop_count:
            rospy.loginfo('已完成 %d 轮巡逻，节点退出。', run)
            break

        rospy.loginfo('循环模式：即将开始下一轮巡逻...')

    if success:
        rospy.loginfo('多点导航节点正常退出。')
    else:
        rospy.logwarn('多点导航节点因错误退出。')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('多点导航节点已被中断。')
