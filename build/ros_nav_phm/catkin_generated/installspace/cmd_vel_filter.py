#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_filter.py

速度代理节点：根据导航节点发布的状态对 move_base 输出的速度做处理。

话题订阅:
  /cmd_vel              (geometry_msgs/Twist)  move_base 输出
  /nav_state            (std_msgs/String)       导航状态

话题发布:
  /cmd_vel_safe         (geometry_msgs/Twist)  底盘实际执行

状态说明:
  NAVIGATING     → 正常导航，速度透传
  WAITING_RED    → 红灯等待，发布 0 速度
  FINISHED       → 导航完成，发布 0 速度
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 状态常量
STATE_NAVIGATING  = "NAVIGATING"
STATE_WAITING_RED = "WAITING_RED"
STATE_FINISHED    = "FINISHED"


class CmdVelFilter:

    def __init__(self):
        rospy.init_node('cmd_vel_filter')

        self.state = STATE_NAVIGATING

        rospy.Subscriber("/cmd_vel",   Twist,  self.cmd_vel_cb,   queue_size=1)
        rospy.Subscriber("/nav_state", String, self.nav_state_cb, queue_size=1)

        self.pub = rospy.Publisher("/cmd_vel_safe", Twist, queue_size=1)

        rospy.loginfo("cmd_vel_filter started.")

    def nav_state_cb(self, msg):
        old_state = self.state
        self.state = msg.data
        # 状态变为 WAITING_RED 或 FINISHED 时，立即发布 0 速度
        if self.state in (STATE_WAITING_RED, STATE_FINISHED):
            if old_state != self.state:
                self.pub.publish(Twist())

    def cmd_vel_cb(self, twist):
        self.pub.publish(self.filter(twist))

    def filter(self, twist):
        # 红灯等待或导航完成时，发布 0 速度
        if self.state in (STATE_WAITING_RED, STATE_FINISHED):
            return Twist()

        # NAVIGATING 状态透传
        return twist


if __name__ == '__main__':
    try:
        CmdVelFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
