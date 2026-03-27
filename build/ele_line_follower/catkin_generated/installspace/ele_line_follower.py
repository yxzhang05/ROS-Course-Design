
#!/usr/bin/env python3
import rospy
import subprocess
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16MultiArray


class EleLineFollower:
    def __init__(self):
        # ===== 最早初始化所有成员，防止 Ctrl+C 炸裂 =====
        self.cmd_pub = None
        self.map_saved = False

        # 循线传感器默认值（防 AttributeError）
        self.L = 0.0
        self.C = 0.0
        self.R = 0.0
        self.last_error = 0.0  

        rospy.init_node("ele_line_follower", anonymous=False)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ele_sub = rospy.Subscriber('/ele_sensor', Int16MultiArray, self.sensor_cb)

        # PID/比例控制参数，可用 rosparam 调节
        self.Kp = rospy.get_param("~Kp", 1.5)
        self.v_forward = rospy.get_param("~v_forward", 0.1)

        # 防止重复保存
        self.map_saved = False

        # Ctrl+C / rosnode kill / roscore shutdown
        rospy.on_shutdown(self.on_shutdown)

        self.rate = rospy.Rate(50)  # 50Hz 循环
        rospy.loginfo("EleLineFollower started. Ctrl+C to stop & save map.")

    def sensor_cb(self, msg):
        # 修正映射：msg.data[0] -> R, msg.data[1] -> C, msg.data[2] -> L
        if len(msg.data) >= 3:
            self.R = msg.data[0]  # 最右侧传感器
            self.C = msg.data[1]  # 中间传感器
            self.L = msg.data[2]  # 最左侧传感器

# ================== Ctrl+C 回调 ==================
    def on_shutdown(self):
        if self.map_saved:
            return

        rospy.logwarn("Shutdown detected! Stop robot and save map...")

        # 停车
        self.stop_robot()

        # 确保 /map 还在（gmapping 仍运行）
        try:
            rospy.wait_for_message("/map", OccupancyGrid, timeout=2.0)
        except rospy.ROSException:
            rospy.logerr("No /map received, map saving aborted")
            return

        # 稍等一会，保证最后一帧写入
        rospy.sleep(0.5)

        # 保存地图
        self.save_map()

        self.map_saved = True
        rospy.loginfo("Shutdown handling finished")

    # ================== 停车 ==================
    def stop_robot(self):
        if self.cmd_pub is None:
            rospy.logwarn("cmd_pub not ready, skip stop")
            return
                
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        for _ in range(5):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

        rospy.loginfo("Robot stopped")

    # ================== 保存地图 ==================
    def save_map(self):
        map_dir = "/home/gdut/catkin_roscar/src/start_roscar/map"
        map_name = "roscar_map"
        map_path = os.path.join(map_dir, map_name)
        # 确保目录存在
        os.makedirs(map_dir, exist_ok=True)

        # 删除旧文件（防止写失败）
        for ext in [".pgm", ".yaml"]:
            f = map_path + ext
            if os.path.exists(f):
                os.remove(f)

        rospy.loginfo("Saving map to %s", map_path)

        subprocess.call([
            "rosrun",
            "map_server",
            "map_saver",
            "-f",
            map_path
        ])

        rospy.loginfo("Map saved successfully")

    def run(self):
        while not rospy.is_shutdown():
            total = self.L + self.C + self.R + 1e-6  # 防止除零
            twist = Twist()
            # 归一化误差：右强 → 右偏 → 角速度为负
            if total < 200:  

                # 丢线：保持转向寻找电磁线
                search_w = 0.6

                if self.last_error > 0:
                    twist.angular.z = search_w
                else:
                    twist.angular.z = -search_w

                twist.linear.x = 0.05   # 慢速寻找

            else:

                # 正常巡线
                error = (self.L - self.R) / (total + 1e-6)

                omega_z = self.Kp * error
                omega_z = max(min(omega_z, 0.8), -0.8)

                twist.linear.x = self.v_forward
                twist.angular.z = omega_z

                self.last_error = error

            if self.cmd_pub:
                self.cmd_pub.publish(twist)
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = EleLineFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass
