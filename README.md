# ROS-Course-Design

本仓库存放 ROS 小车课程设计的代码，包含底盘驱动、SLAM 建图、自主导航以及多点导航等功能。

---

## 仓库目录结构

```
src/
├── start_roscar/          # 小车启动、地图、导航参数配置包
│   ├── launch/            # 启动文件（navigation.launch 等）
│   ├── map/               # 已建好的地图（roscar_map1.pgm / .yaml 等）
│   └── param/             # move_base、costmap、规划器参数
└── wulina/                # ★ 新增功能包（多点导航等新代码放这里）
    ├── scripts/
    │   └── multi_point_navigation.py   # 多点导航节点
    └── launch/
        └── multi_point_navigation.launch  # 一键启动多点导航
```

---

## 如何将 wulina 放到小车上

> **重要**：`wulina` 必须放在 catkin 工作空间的 `src/` 目录下，  
> **不能**放在其他功能包（如 `traffic_light_yolo`）的子目录里。

```bash
# 正确位置（以小车的工作空间 catkin_roscar 为例）
catkin_roscar/
└── src/
    ├── start_roscar/
    ├── traffic_light_yolo/
    └── wulina/          ← 放这里
```

将本仓库的 `src/wulina/` 整个文件夹复制到小车的 `catkin_roscar/src/` 下：

```bash
# 在小车上执行（假设已通过 git clone 或 scp 获取代码）
cp -r /path/to/ROS-Course-Design/src/wulina  ~/catkin_roscar/src/

# 重新编译工作空间
cd ~/catkin_roscar
catkin_make

# 刷新环境变量
source devel/setup.bash
```

---

## 如何启动多点导航

### 方式一：一键启动（推荐）

该方式会同时启动底盘、地图、AMCL 定位、move_base 和多点导航节点：

```bash
roslaunch wulina multi_point_navigation.launch
```

可选参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `map_file` | 地图 yaml 文件路径 | `start_roscar/map/roscar_map1.yaml` |
| `loop` | 是否循环巡逻 | `false` |
| `loop_count` | 循环次数（`0` = 无限） | `1` |
| `goal_timeout` | 单个目标超时（秒） | `60.0` |

示例——循环巡逻（无限次）：

```bash
roslaunch wulina multi_point_navigation.launch loop:=true loop_count:=0
```

示例——使用其他地图：

```bash
roslaunch wulina multi_point_navigation.launch \
  map_file:=$(rospack find start_roscar)/map/roscar_map2.yaml
```

### 方式二：导航栈已运行时单独启动节点

如果 `navigation.launch` 已经在另一个终端运行，可以单独启动导航节点：

```bash
rosrun wulina multi_point_navigation.py
```

---

## 如何修改导航点坐标

打开 `src/wulina/scripts/multi_point_navigation.py`，找到顶部的 `WAYPOINTS` 列表：

```python
# ★ 航点配置区域 ★ — 修改这里的坐标即可
WAYPOINTS = [
    {'name': '航点1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
    {'name': '航点2', 'x': 1.0, 'y': 1.0, 'yaw': math.pi / 2},
    {'name': '航点3', 'x': 0.0, 'y': 1.0, 'yaw': math.pi},
    {'name': '航点4（返回起点）', 'x': 0.0, 'y': 0.0, 'yaw': -math.pi / 2},
]
```

- `x` / `y`：目标点在地图坐标系下的坐标，单位为**米**。
- `yaw`：到达后的朝向角，单位为**弧度**（0 = X 轴正方向，`math.pi/2` ≈ 90°）。
- 可以用 RViz 的「2D Nav Goal」工具点击地图，终端会打印出坐标，将其填入列表即可。
- 增减航点只需在列表中添加或删除字典条目。

---

## 常用启动命令汇总

```bash
# 1. 启动小车底盘和激光雷达
roslaunch start_roscar start_roscar.launch

# 2. 启动电磁巡线节点
rosrun ele_line_follower ele_line_follower.py

# 3. 启动 SLAM 建图
roslaunch roscar_slam gmapping.launch

# 4. 启动导航（可用 RViz 手动指定目标）
roslaunch start_roscar navigation.launch

# 5. 启动多点自动导航（wulina 包）
roslaunch wulina multi_point_navigation.launch
```
