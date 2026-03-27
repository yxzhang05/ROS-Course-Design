# 精准测距算法升级说明文档

## 概述

本次升级将测距算法从**单点索引查找**改回经过增强的**窗口最小值策略**，并在灵敏度、角度跟踪、时间同步等方面全面优化。同时，导航状态机新增了**基于实时距离的动态停车逻辑**，使小车能够在距交通灯约 1m 处精准停下，不再依赖预先设定的坐标停止点。

---

## 一、改动文件清单

| 文件路径 | 改动内容 |
|---|---|
| `src/traffic_light_yolo_1/src/traffic_light_yolov8_node.py` | 重构 `_calc_distance` 为窗口最小值策略；新增置信度过滤；优化去抖/滤波参数 |
| `src/ros_nav_phm/scripts/traffic_light_navigator.py` | 新增基于距离的动态停车逻辑；更新 `_detection_cb` 读取距离；修复 PREEMPTED 状态处理 |
| `src/ros_nav_phm/launch/roscar_nav.launch` | 新增 `stop_distance`、`action_threshold` 参数 |
| `src/traffic_light_yolo_1/param/detector_params.yaml` | 新增 `scan_window_half`、`conf_threshold` 参数；更新所有参数默认值 |

---

## 二、测距算法对比

### 旧算法（单点索引查找）

```python
# 将视觉角度直接换算为雷达数组索引，取该单个点的距离
angle_rad = math.radians(angle_deg + self.angle_offset)
index = int((angle_rad - scan.angle_min) / scan.angle_increment)
distance = scan.ranges[index]
```

**问题**：
- 单点命中率低：角度计算误差、雷达分辨率限制导致该点常返回 inf 或反射失效值
- 无容错机制：单点无效直接丢弃本帧测距，距离输出频繁出现 inf
- 对 `camera_fov` / `angle_offset` 标定精度要求极高

### 当前算法（窗口最小值策略）

```python
# 在目标角度 ± scan_window_half 个索引范围内，取所有有效点的最小值
center = int((angle_rad - scan.angle_min) / scan.angle_increment)
start = max(0, center - self.scan_window_half)
end   = min(n, center + self.scan_window_half + 1)
valid = [scan.ranges[i] for i in range(start, end)
         if scan.range_min <= scan.ranges[i] <= scan.range_max]
return min(valid)  # 窗口内最近障碍物即为灯体/灯杆本身
```

**优势**：
- 取最小值而非中值：灯体是方向上最近的实体，最小值直接命中目标，精度优于中值
- 窗口容错：单点失效时邻近点自动兜底，大幅减少 inf 输出
- `scan_window_half` 可调（默认5），平衡精度与容错

**注意**：窗口不宜过大（> 10），否则会把灯旁边的墙壁/柱子也纳入，导致取最小值偏小。

---

## 三、停车逻辑升级

### 旧逻辑（基于 waypoint 坐标触发）

1. 在 `waypoints.yaml` 中预设红绿灯停止点的坐标（`traffic_light_stop_points`）
2. 小车到达该坐标点后，检查灯色
3. 红灯则等待，绿灯则发下一个导航点

**局限性**：停止位置完全取决于坐标设置精度，无法自适应

### 新逻辑（基于实时距离触发，主要）+ 备用 waypoint 触发

```
NAVIGATING 状态:
  ├─ 红灯 AND 距离 < action_threshold (1.5m) → 取消当前导航目标 → 进入 WAITING_RED
  └─ 到达预设停止点 waypoint AND 红灯 → 进入 WAITING_RED（备用）

WAITING_RED 状态:
  └─ 绿灯 → 回到 NAVIGATING → 重发被中断的 waypoint，继续行驶
```

**改进效果**：
- 无论小车在哪个位置，只要检测到红灯且测距 < 1.5m，立即停车
- 停车位置取决于实际距离，不依赖预设坐标
- 绿灯后自动从中断点继续导航，不丢失当前目标

---

## 四、使用教程

### 4.1 参数校准（重要！）

测距精度直接依赖两个参数的准确性：

**① `camera_fov`（摄像头水平视场角）**

方法一：查摄像头规格书
- 本项目默认值：93.4°

方法二：用相机标定工具获取精确值
```bash
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.025 image:=/usb_cam/image_raw
# 标定完成后，用内参 fx 计算：
# camera_fov = 2 × arctan(image_width / (2 × fx)) × (180/π)
```

**② `angle_offset`（摄像头与雷达安装偏角补偿）**

校准步骤：
1. 将小车对准交通灯，使交通灯出现在画面正中间
2. 启动检测节点：`rosrun traffic_light_yolo_1 traffic_light_yolov8_node.py`
3. 查看终端输出的 `Angle` 和 `Distance` 字段，与卷尺测量值对比
4. 在 `detector_params.yaml` 中调整 `angle_offset`：
   - 测量距离偏大（雷达打到了背景）→ 调大/调小 `angle_offset` 直到指向灯体
   - 每次调整后重启节点验证

修改参数文件位置：
```
src/traffic_light_yolo_1/param/detector_params.yaml
```

### 4.2 停车距离调整

在 `roscar_nav.launch` 中调整：

```xml
<param name="stop_distance"    value="1.0"/>  <!-- 期望停在距交通灯多远处 -->
<param name="action_threshold" value="1.5"/>  <!-- 检测到红灯时多远开始停车 -->
```

建议 `action_threshold = stop_distance + 0.5`，给刹车留出缓冲。

### 4.3 完整启动流程

**步骤 1：加载环境**
```bash
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
```

**步骤 2：启动导航（雷达建图 + AMCL 定位 + move_base）**
```bash
roslaunch start_roscar navigation.launch \
  map_file:=/home/gdut/catkin_roscar/src/start_roscar/map/roscar_map_2.yaml
```

**步骤 3：启动红绿灯检测节点（含新测距算法）**
```bash
rosrun traffic_light_yolo_1 traffic_light_yolov8_node.py
```

**步骤 4：启动多点导航 + 红绿灯控制节点**
```bash
roslaunch ros_nav_phm roscar_nav.launch
```

---

## 五、调试方法

**实时查看检测结果和距离**：
```bash
rostopic echo /traffic_light_detection
# data: [state, angle_deg, distance_m]
# state: 0=无灯, 1=红灯, 2=绿灯
```

**实时查看导航状态**：
```bash
rostopic echo /nav_state
# NAVIGATING / WAITING_RED / FINISHED
```

**可视化雷达数据（用于校准角度）**：
```bash
rviz  # 添加 LaserScan 话题 /scan，观察交通灯对应的雷达点
```

**验证测距准确性**：
1. 将小车放在距交通灯 2.0m 处（卷尺量）
2. 执行 `rostopic echo /traffic_light_detection`
3. `data[2]` 应接近 2.0m（误差 < ±0.15m 为合格）
4. 若误差大，校准 `camera_fov` 和 `angle_offset`

---

## 六、waypoints.yaml 配置建议

由于新增了基于距离的动态停车，`traffic_light_stop_points` 变为备用机制。原有配置不需要修改，但如果不再需要 waypoint 触发，可将其留空：

```yaml
# 保留原有配置，作为测距失败时的备用触发机制
traffic_light_stop_points: [8, 11]

# 或者清空，完全依赖距离触发
# traffic_light_stop_points: []
```

**推荐保留**：当小车距离交通灯很近但雷达无法测量时（如交通灯在雷达盲区），waypoint 触发可作为保底机制。
