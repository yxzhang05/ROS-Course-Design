#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import String
import tf
import math

class MultiGoalNavigation:
    def __init__(self):
        rospy.init_node('multi_goal_navigation')
        
        # 创建action客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.client.wait_for_server()
        rospy.loginfo("连接到move_base服务器成功！")
        
        # 定义导航点列表 - 从RViz获取的坐标
        # 格式: [名称, x坐标, y坐标, 朝向角度(弧度)]
        self.goal_points = [
            # 你可以从RViz获取坐标后填入这里
            ["起点", 1.440, 3.229, 1.161],
            ["点1", 1.063, 4.770, 1.161],  # 朝向90度
            ["点2", 1.046, 4.791, 3.14], # 朝向180度
            ["点3", 0.759, 4.838, -1.57],# 朝向-90度
            ["终点", 1.490, 4.811, 0.0],
        ]
        
        self.current_goal_index = 0
        self.total_goals = len(self.goal_points)
        
        # 订阅红绿灯状态（可选，用于与红绿灯检测集成）
        rospy.Subscriber('/traffic_light_state', String, self.traffic_light_callback)
        self.traffic_state = "none"
        
        rospy.loginfo("多点导航节点初始化完成")
        rospy.loginfo(f"共设置 {self.total_goals} 个导航点")
    
    def traffic_light_callback(self, msg):
        """红绿灯状态回调"""
        self.traffic_state = msg.data
        if self.traffic_state == "red":
            rospy.logwarn("检测到红灯，等待中...")
            # 如果需要红灯停车，可以在这里处理
            # 注意：move_base会自动处理障碍物，所以不一定需要额外处理
    
    def create_goal(self, x, y, yaw):
        """创建导航目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position = Point(x, y, 0.0)
        
        # 设置朝向（将yaw角度转换为四元数）
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        return goal
    
    def send_next_goal(self):
        """发送下一个导航点"""
        if self.current_goal_index < self.total_goals:
            goal_name, x, y, yaw = self.goal_points[self.current_goal_index]
            
            rospy.loginfo(f"正在前往第 {self.current_goal_index + 1}/{self.total_goals} 个点: {goal_name}")
            rospy.loginfo(f"坐标: x={x:.2f}, y={y:.2f}, 朝向={yaw:.2f}弧度")
            
            goal = self.create_goal(x, y, yaw)
            
            # 发送目标并设置回调
            self.client.send_goal(goal, self.goal_done_callback)
        else:
            rospy.loginfo("所有导航点已完成！")
    
    def goal_done_callback(self, status, result):
        """导航完成回调"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"✅ 到达目标点: {self.goal_points[self.current_goal_index][0]}")
            self.current_goal_index += 1
            
            # 继续发送下一个目标
            if self.current_goal_index < self.total_goals:
                rospy.sleep(2)  # 停留2秒
                self.send_next_goal()
            else:
                rospy.loginfo("🎉 所有导航任务完成！")
        else:
            rospy.logwarn(f"❌ 导航失败，状态码: {status}")
    
    def run(self):
        """开始导航"""
        rospy.loginfo("开始多点导航...")
        rospy.sleep(2)  # 等待系统稳定
        self.send_next_goal()
        
        # 保持节点运行
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = MultiGoalNavigation()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航节点被中断")