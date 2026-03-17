cmake_minimum_required(VERSION 3.0.2)
project(start_roscar)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

## 安装 Python 脚本，使其可通过 rosrun 直接调用
catkin_install_python(PROGRAMS
  scripts/multi_point_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
