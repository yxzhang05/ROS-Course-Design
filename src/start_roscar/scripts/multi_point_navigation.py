#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多点导航节点
============
本节点基于 ROS move_base 动作服务器，实现机器人按照预设的航点列表依次导航。

使用说明：
  1. 确保 navigation.launch 已启动（底盘、地图服务器、AMCL、move_base）。
  2. 修改下方 WAYPOINTS 列表中的坐标与偏航角，以适配实际地图中的目标点。
  3. 通过 multi_point_navigation.launch 启动本节点，或直接运行：
       rosrun start_roscar multi_point_navigation.py

参数说明（可通过 ROS 参数服务器覆盖）：
  ~loop          (bool,  默认: False)  — 是否循环执行航点列表
  ~loop_count    (int,   默认: 1)     — 循环次数（loop=False 时忽略）；设为 0 表示无限循环
  ~goal_timeout  (float, 默认: 60.0)  — 单个目标的超时时间（秒）
  ~map_frame     (str,   默认: "map") — 导航坐标系
"""

import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# ==============================================================================
#  ★ 航点配置区域 ★
#  请在此处修改导航点的坐标，格式说明：
#    'name' : 航点名称（用于日志显示，可随意命名）
#    'x'    : 目标点在地图坐标系下的 X 坐标（单位：米）
#    'y'    : 目标点在地图坐标系下的 Y 坐标（单位：米）
#    'yaw'  : 到达目标点后的偏航角（单位：弧度；0 = X轴正方向，π/2 = Y轴正方向）
#             可使用 math.pi/2、math.pi、-math.pi/2 等表达式
#
#  提示：可在 RViz 中使用"2D Nav Goal"工具点击地图，终端会输出目标点坐标，
#  将坐标值填写到下方列表即可。
# ==============================================================================
WAYPOINTS = [
    {
        'name': '航点1',
        'x':    1.0,    # X 坐标（米）
        'y':    0.0,    # Y 坐标（米）
        'yaw':  0.0,    # 偏航角（弧度）
    },
    {
        'name': '航点2',
        'x':    1.0,    # X 坐标（米）
        'y':    1.0,    # Y 坐标（米）
        'yaw':  math.pi / 2,  # 偏航角（弧度），π/2 ≈ 90°
    },
    {
        'name': '航点3',
        'x':    0.0,    # X 坐标（米）
        'y':    1.0,    # Y 坐标（米）
        'yaw':  math.pi,  # 偏航角（弧度），π ≈ 180°
    },
    {
        'name': '航点4（返回起点）',
        'x':    0.0,    # X 坐标（米）
        'y':    0.0,    # Y 坐标（米）
        'yaw':  -math.pi / 2,  # 偏航角（弧度），-π/2 ≈ -90°
    },
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


def navigate_to_waypoints(client, waypoints, map_frame, goal_timeout):
    """
    按顺序导航到航点列表中的每个目标点。

    参数:
        client       (SimpleActionClient): move_base 动作客户端
        waypoints    (list):              航点字典列表
        map_frame    (str):               目标坐标系名称
        goal_timeout (float):             单个目标的超时时间（秒）

    返回:
        bool: 所有航点均成功到达时返回 True，否则返回 False
    """
    total = len(waypoints)

    for idx, wp in enumerate(waypoints):
        # 检查节点是否已被关闭
        if rospy.is_shutdown():
            rospy.logwarn('节点已关闭，停止导航。')
            return False

        name = wp.get('name', '未命名航点')
        rospy.loginfo('─' * 50)
        rospy.loginfo('正在前往第 %d/%d 个航点：%s', idx + 1, total, name)
        rospy.loginfo('  目标坐标：x=%.3f  y=%.3f  yaw=%.3f rad',
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

    return True


def main():
    """
    多点导航主函数。
    初始化 ROS 节点，连接 move_base 动作服务器，然后按配置执行导航循环。
    """
    # 初始化 ROS 节点，节点名称为 multi_point_navigation
    rospy.init_node('multi_point_navigation', anonymous=False)

    # ------------------------------------------------------------------
    # 从参数服务器读取运行参数（可在 launch 文件或命令行中覆盖）
    # ------------------------------------------------------------------
    # 是否循环执行所有航点
    loop = rospy.get_param('~loop', False)
    # 循环次数（loop=False 时忽略；loop=True 时有效；0 表示无限循环）
    loop_count = rospy.get_param('~loop_count', 1)
    # 单个目标的最长等待时间（秒）
    goal_timeout = rospy.get_param('~goal_timeout', 60.0)
    # 导航坐标系（与地图坐标系一致，通常为 "map"）
    map_frame = rospy.get_param('~map_frame', 'map')

    rospy.loginfo('=' * 50)
    rospy.loginfo('多点导航节点已启动')
    rospy.loginfo('  航点数量  : %d', len(WAYPOINTS))
    rospy.loginfo('  循环模式  : %s', '开启' if loop else '关闭')
    if loop:
        rospy.loginfo('  循环次数  : %s', '无限' if loop_count == 0 else str(loop_count))
    rospy.loginfo('  目标超时  : %.1f 秒', goal_timeout)
    rospy.loginfo('  坐标系    : %s', map_frame)
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
    run = 0         # 已完成的完整巡逻轮次
    success = True  # 记录最终结果

    while not rospy.is_shutdown():
        run += 1
        rospy.loginfo('━' * 50)
        if loop:
            rospy.loginfo('开始第 %d 轮巡逻（共 %s 轮）',
                          run, '无限' if loop_count == 0 else str(loop_count))
        else:
            rospy.loginfo('开始单次多点导航任务')

        # 执行一轮所有航点的导航
        success = navigate_to_waypoints(client, WAYPOINTS, map_frame, goal_timeout)

        if not success:
            rospy.logwarn('本轮导航未能完成所有航点，节点退出。')
            break

        rospy.loginfo('━' * 50)
        rospy.loginfo('✓ 已完成所有 %d 个航点的导航。', len(WAYPOINTS))

        # 判断是否需要继续循环
        if not loop:
            # 非循环模式：执行完一轮后退出
            rospy.loginfo('非循环模式，导航任务已全部完成。节点退出。')
            break

        if loop_count != 0 and run >= loop_count:
            # 有限循环模式：已达到指定轮次后退出
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
