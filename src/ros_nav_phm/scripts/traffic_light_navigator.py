#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light_navigator.py

多点导航 + 红绿灯状态机控制节点

状态机:
  NAVIGATING      → 正常导航
  WAITING_RED     → 红灯 + 到达停止点，等待绿灯
  FINISHED        → 完成所有 waypoint

并发模型（纯 ROS 回调，无手动线程）:
  rospy.Timer (10Hz)  → 驱动状态机
  move_base done_cb   → 到达目标点后发下一个点

话题订阅:
  /traffic_light_detection  (std_msgs/Float32MultiArray)
      data: [state(0=none,1=red,2=green), angle(deg), distance(m)]

话题发布:
  /nav_state                (std_msgs/String)   当前状态，供 cmd_vel_filter 使用

使用方法:
  1. 在 src/roscar_nav/param/waypoints.yaml 中填入实际坐标
  2. 在 waypoints.yaml 中配置 traffic_light_stop_points 指定红绿灯停止点索引
  3. roslaunch roscar_nav roscar_nav.launch
"""

import math
import collections

import rospy
import actionlib

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# ============================================================
# 状态常量
# ============================================================
STATE_NAVIGATING  = "NAVIGATING"
STATE_WAITING_RED = "WAITING_RED"
STATE_FINISHED    = "FINISHED"

LIGHT_NONE  = 0
LIGHT_RED   = 1
LIGHT_GREEN = 2


class TrafficLightNavigator:

    def __init__(self):
        rospy.init_node('traffic_light_navigator')

        # 参数
        self.detection_timeout = rospy.get_param("~detection_timeout", 5.0)

        # 距离触发停车参数（来自 navigation_with_traffic_light.py 的精准测距逻辑）
        self.stop_distance    = rospy.get_param("~stop_distance",    1.0)   # 目标停止距离（米）
        self.action_threshold = rospy.get_param("~action_threshold", 1.5)   # 触发刹车的距离阈值（米）

        # 状态机
        self.state = STATE_NAVIGATING
        self.ready = False  # __init__ 成功完成后置 True

        # 红绿灯检测
        self.last_detection_time = None
        self.light_state         = LIGHT_NONE
        self.light_distance      = float('inf')   # 由检测节点测距后发布，单位：米

        # 红绿灯停止点配置（waypoint 索引列表）
        if not rospy.has_param("~traffic_light_stop_points"):
            rospy.logwarn("~traffic_light_stop_points not set, no traffic light control.")
            self.traffic_light_stop_points = []
        else:
            self.traffic_light_stop_points = rospy.get_param("~traffic_light_stop_points")
            rospy.loginfo("Traffic light stop points: %s", self.traffic_light_stop_points)

        # 从参数服务器加载路径配置（由 waypoints.yaml 提供）
        if not rospy.has_param("~waypoints"):
            rospy.logfatal("~waypoints not found. Load src/roscar_nav/param/waypoints.yaml first.")
            rospy.signal_shutdown("missing ~waypoints")
            return
        waypoints_raw  = rospy.get_param("~waypoints")
        via_points_raw = rospy.get_param("~via_points", [])

        # 导航点队列
        self.waypoints  = collections.deque(waypoints_raw)
        self.via_points = via_points_raw
        self.wp_index        = 0      # 当前目标点序号（用于日志）
        self.goal_active     = False  # 是否有目标正在执行
        self.current_wp      = None   # 当前正在执行的目标点（用于失败重试）
        self.retry_count     = 0      # 当前点重试次数
        self.max_retries     = rospy.get_param("~max_retries", 3)
        self.pending_next    = False  # done_cb 请求发下一个点，由 timer_cb 执行

        # 当前是否为红绿灯停止点（到达后才置 True）
        self.is_at_traffic_light_stop = False
        self.state_pub = rospy.Publisher("/nav_state", String, queue_size=1, latch=True)
        self.via_pub   = rospy.Publisher(
            "/move_base/TebLocalPlannerROS/via_points", Path, queue_size=1, latch=True)

        # move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logfatal("move_base not available after 10s, shutting down.")
            rospy.signal_shutdown("move_base unavailable")
            return
        rospy.loginfo("move_base connected.")

        # 订阅
        rospy.Subscriber("/traffic_light_detection", Float32MultiArray,
                         self._detection_cb, queue_size=1)

        # 10Hz 定时器驱动状态机
        rospy.Timer(rospy.Duration(0.1), self._timer_cb)
        self.ready = True

    # ----------------------------------------------------------
    # 回调：检测红绿灯
    # ----------------------------------------------------------
    def _detection_cb(self, msg):
        if len(msg.data) < 1:
            return
        self.last_detection_time = rospy.Time.now()
        self.light_state = int(msg.data[0])
        if len(msg.data) >= 3:
            self.light_distance = float(msg.data[2])

    def _set_state(self, new_state):
        if self.state == new_state:
            return
        rospy.loginfo("State: %s -> %s", self.state, new_state)
        self.state = new_state
        self.state_pub.publish(String(data=new_state))

    def _make_goal(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        if "yaw" in wp:
            yaw = wp["yaw"]
            goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            # 不指定朝向，TEB 会根据 global_plan_overwrite_orientation 自动计算
            goal.target_pose.pose.orientation.w = 1.0
        return goal

    def _send_via_points(self, index):
        """发布当前段的 via-points 给 TEB 局部规划器，无 via-points 时发空路径清除残留"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = rospy.Time.now()
        if index < len(self.via_points) and self.via_points[index]:
            for pt in self.via_points[index]:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp    = rospy.Time.now()
                pose.pose.position.x = pt["x"]
                pose.pose.position.y = pt["y"]
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            rospy.loginfo("Via-points for waypoint %d published (%d points)", index, len(path.poses))
        self.via_pub.publish(path)

    def _send_next(self):
        """从队列取下一个点并发送，队列空则完成"""
        if self.goal_active:
            rospy.logwarn("_send_next called while goal is still active, ignoring.")
            return
        if not self.waypoints:
            self._set_state(STATE_FINISHED)
            rospy.loginfo("All waypoints reached.")
            return
        wp = self.waypoints.popleft()
        self.current_wp  = wp
        self.retry_count = 0

        self._send_via_points(self.wp_index)
        rospy.loginfo("Sending waypoint %d: x=%.2f y=%.2f",
                      self.wp_index, wp["x"], wp["y"])
        self.client.send_goal(self._make_goal(wp), done_cb=self._done_cb)
        self.goal_active = True
        self.wp_index   += 1

    # ----------------------------------------------------------
    # 回调：move_base 完成
    # ----------------------------------------------------------
    def _done_cb(self, status, result):
        self.goal_active = False
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint %d", self.wp_index - 1)
            reached_index = self.wp_index - 1
            # 判断是否是红绿灯停止点（备用：waypoint 坐标触发）
            if reached_index in self.traffic_light_stop_points:
                # 是停止点：不立即发下一个点，先检查灯态
                self.is_at_traffic_light_stop = True
                # 在 timer_cb 中检查红灯，如果是红灯则进入 WAITING_RED，否则发下一个点
            else:
                # 不是停止点，直接发下一个点
                self.pending_next = True
        elif status == GoalStatus.PREEMPTED and self.state == STATE_WAITING_RED:
            # 红灯停车时主动取消目标，不做重试，等绿灯后由 timer_cb 恢复
            rospy.loginfo("Goal preempted for red light stop at waypoint %d.", self.wp_index - 1)
        else:
            self.retry_count += 1
            if self.retry_count <= self.max_retries:
                rospy.logwarn("move_base failed (status=%d), retry %d/%d in 1s...",
                              status, self.retry_count, self.max_retries)
                rospy.Timer(rospy.Duration(1.0), self._retry_cb, oneshot=True)
            else:
                rospy.logerr("Waypoint %d failed after %d retries, skipping.",
                             self.wp_index - 1, self.max_retries)
                self.pending_next = True

    def _retry_cb(self, event):
        """重试当前点，不重置 retry_count"""
        rospy.loginfo("Retrying waypoint %d: x=%.2f y=%.2f",
                      self.wp_index - 1, self.current_wp["x"], self.current_wp["y"])
        self._send_via_points(self.wp_index - 1)
        self.client.send_goal(self._make_goal(self.current_wp), done_cb=self._done_cb)
        self.goal_active = True

    # ----------------------------------------------------------
    # 定时器回调（10Hz）：驱动状态机
    # ----------------------------------------------------------
    def _timer_cb(self, event):
        if self.state == STATE_FINISHED:
            return

        # 延迟发送下一个目标点（不在 WAITING_RED 时处理，避免跳过当前未完成的 waypoint）
        if self.pending_next and self.state != STATE_WAITING_RED:
            self.pending_next = False
            self._send_next()
            return

        # 超时则视为无检测
        if self.last_detection_time is None or \
           (rospy.Time.now() - self.last_detection_time).to_sec() > self.detection_timeout:
            light = LIGHT_NONE
        else:
            light = self.light_state

        is_red   = (light == LIGHT_RED)
        is_green = (light == LIGHT_GREEN)

        # 距离是否在停车触发阈值以内（有效距离 + 小于 action_threshold）
        within_stopping_distance = (self.light_distance != float('inf') and
                                    self.light_distance < self.action_threshold)

        # 状态转换
        if self.state == STATE_NAVIGATING:
            # 【精准测距触发】红灯 + 距离 < action_threshold → 立即停车
            if is_red and within_stopping_distance:
                rospy.loginfo("RED light at %.2fm (< %.2fm threshold), stopping.",
                              self.light_distance, self.action_threshold)
                self._set_state(STATE_WAITING_RED)
                # 取消当前 move_base 目标，防止导航超时自动重试
                if self.goal_active:
                    self.client.cancel_all_goals()
                    self.goal_active = False
                return

            # 【备用：waypoint 坐标触发】到达预设停止点后检查灯态
            if self.is_at_traffic_light_stop:
                if is_red:
                    self._set_state(STATE_WAITING_RED)
                else:
                    rospy.loginfo("At stop point but light is green/none, continuing.")
                    self.is_at_traffic_light_stop = False
                    self.pending_next = True

        elif self.state == STATE_WAITING_RED:
            if is_green:
                rospy.loginfo("GREEN — resuming navigation.")
                self.is_at_traffic_light_stop = False
                self._set_state(STATE_NAVIGATING)
                # 恢复：重发当前 waypoint（被红灯中断时 current_wp 由 _send_next 保证已设置）
                # 若 current_wp 为 None（异常情况），则退化为发下一个点
                if self.current_wp is not None and not self.goal_active:
                    self.retry_count = 0
                    self._send_via_points(self.wp_index - 1)
                    self.client.send_goal(self._make_goal(self.current_wp),
                                          done_cb=self._done_cb)
                    self.goal_active = True
                    rospy.loginfo("Resuming waypoint %d after green light.", self.wp_index - 1)
                else:
                    self.pending_next = True

    # ----------------------------------------------------------
    # 启动
    # ----------------------------------------------------------
    def run(self):
        if not self.ready:
            rospy.logerr("Initialization failed, node will not start.")
            return
        self.state_pub.publish(String(data=self.state))
        self._send_next()   # 发第一个点
        rospy.spin()


if __name__ == '__main__':
    try:
        TrafficLightNavigator().run()
    except rospy.ROSInterruptException:
        pass
