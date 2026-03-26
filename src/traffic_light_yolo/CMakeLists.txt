cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

# 安装 Python 脚本（赋予可执行权限）
catkin_install_python(PROGRAMS
  scripts/traffic_light_yolov8_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
