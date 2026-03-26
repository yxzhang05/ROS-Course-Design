cmake_minimum_required(VERSION 3.0.2)
project(wulina)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

## 安装 Python 脚本，使其可通过 rosrun wulina <脚本名> 调用
catkin_install_python(PROGRAMS
  scripts/multi_point_navigation.py
  scripts/straight_line_nav.py
  scripts/odom_navi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch 文件，使其可通过 roslaunch wulina <launch文件名> 调用
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装 param 文件，使其可通过 $(find wulina)/param/ 访问
install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)
