# 指定最低的 CMake 版本要求
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称，必须和 package.xml 中的 <name> 一致
project(wulina)

# 查找 Catkin 编译所需的依赖项
# 因为我们在创建包时指定了 rospy，所以这里要找到它
find_package(catkin REQUIRED COMPONENTS
  rospy
)

# 声明包的信息
# 这对于纯Python包来说比较简单
catkin_package()

# (核心步骤) 安装Python可执行脚本
# 这条命令告诉 catkin_make：
# “把 scripts 文件夹下的这两个文件安装成可执行程序”
# 这样 rosrun 和 roslaunch 才能找到它们。
install(PROGRAMS
  scripts/multi_point_navigation.py
  scripts/multi_point_navigation1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)