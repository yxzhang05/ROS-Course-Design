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

参数说明（可通过 ROS 参数服务器或 launch 文件覆盖）：
  ~loop          (bool,  默认: False)  — 是否循环执行航点列表
  ~loop_count    (int,   默认: 1)     — 循环次数（loop=False 时忽略）；设为 0 表示无限循环
  ~goal_timeout  (float, 默认: 60.0)  — 单个目标的超时时间（秒）
  ~map_frame     (str,   默认: "map") — 导航坐标系
  ~interactive   (bool,  默认: True)  — 是否在每步之间显示终端交互提示
"""

import sys
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# ==============================================================================
#  ★ 航点配置区域 ★
#  格式说明：
#    'name' : 航点名称（仅用于日志显示）
#    'x'    : 目标点在地图坐标系下的 X 坐标（米）
#    'y'    : 目标点在地图坐标系下的 Y 坐标（米）
#    'yaw'  : 到达目标点后的偏航角（弧度）
#             正值 = 逆时针，负值 = 顺时针；0 = X 轴正方向
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
        except EOFError:
            sys.stdout.write('\n（检测到非交互式输入，使用默认值 {:.4f}）\n'.format(default))
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
                           interactive=True):
    """
    按顺序导航到航点列表中的每个目标点。
    当 interactive=True 时：
      - 执行第一个航点前，先在终端显示起始航点坐标并等待用户确认/修改/停止。
      - 每到达一个航点（最后一个除外），在终端显示下一个航点信息并等待用户指令。

    参数:
        client       (SimpleActionClient): move_base 动作客户端
        waypoints    (list):              航点字典列表
        map_frame    (str):               目标坐标系名称
        goal_timeout (float):             单个目标的超时时间（秒）
        interactive  (bool):              是否启用交互式终端控制

    返回:
        bool: 所有航点均成功到达时返回 True，否则返回 False
    """
    total = len(waypoints)

    # 将原始列表复制一份，以便交互模式下可对航点临时修改而不影响原列表
    pending = [dict(wp) for wp in waypoints]
    idx = 0

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
        rospy.loginfo('  目标坐标：x=%.4f  y=%.4f  yaw=%.4f rad',
                      wp['x'], wp['y'], wp['yaw'])

        # 构造并发送目标
        goal = build_goal(wp, map_frame)
        client.send_goal(goal)

        # 等待结果，设置超时
        finished = client.wait_for_result(rospy.Duration(goal_timeout))

        if not finished:
            # 超时：取消当前目标并报错
            rospy.logwarn('航点 [%s] 导航超时（%.1f 秒），取消当前目标。', name, goal_timeout)
            client.cancel_goal()
            return False

        # 获取最终状态
        state = client.get_state()
        # actionlib.GoalStatus.SUCCEEDED == 3
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('✓ 成功到达航点 [%s]', name)
        else:
            rospy.logwarn('✗ 未能到达航点 [%s]，状态码：%d', name, state)
            return False

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
    loop = rospy.get_param('~loop', False)
    loop_count = rospy.get_param('~loop_count', 1)
    goal_timeout = rospy.get_param('~goal_timeout', 60.0)
    map_frame = rospy.get_param('~map_frame', 'map')
    # 是否启用交互式终端控制（每步之间在终端提示用户）
    interactive = rospy.get_param('~interactive', True)

    rospy.loginfo('=' * 50)
    rospy.loginfo('多点导航节点已启动')
    rospy.loginfo('  航点数量  : %d', len(WAYPOINTS))
    rospy.loginfo('  循环模式  : %s', '开启' if loop else '关闭')
    if loop:
        rospy.loginfo('  循环次数  : %s', '无限' if loop_count == 0 else str(loop_count))
    rospy.loginfo('  目标超时  : %.1f 秒', goal_timeout)
    rospy.loginfo('  坐标系    : %s', map_frame)
    rospy.loginfo('  交互控制  : %s', '开启' if interactive else '关闭')
    rospy.loginfo('=' * 50)

    # ------------------------------------------------------------------
    # 检查航点列表是否为空
    # ------------------------------------------------------------------
    if not WAYPOINTS:
        rospy.logerr('航点列表为空，请在脚本顶部的 WAYPOINTS 列表中添加导航点。')
        return

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
            client, WAYPOINTS, map_frame, goal_timeout, interactive)

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
