# 直线导航模块说明（straight_line_navigation）

> 对应 GitHub issue：小车多点导航时无故后退，且路径不走直线。  
> 本文档说明新增文件的作用、使用方法及可调参数。

---

## 一、问题背景

原始 `navigation.launch` 使用 TEB 局部规划器，TEB 默认按阿克曼底盘模型优化
（`min_turning_radius: 0.750`、`wheelbase: 0.322`），且允许后退
（`max_vel_x_backwards: 0.15`）。在无障碍物的开阔场景中，TEB 仍可能规划出
后退或弧形路径，原因是：

1. 阿克曼约束导致 TEB 需要通过倒退来满足最小转弯半径。  
2. 路径长度权重为 0（`weight_shortest_path: 0`），TEB 不会主动选最短直线路径。  
3. `global_plan_overwrite_orientation: False` 使 TEB 不跟随全局直线路径的朝向，
   进而生成弯曲轨迹。

---

## 二、新增文件一览

| 文件路径 | 说明 |
|---|---|
| `src/wulina/launch/straight_line_navigation.launch` | 新导航启动文件（替代 navigation.launch 使用） |
| `src/wulina/param/straight_teb_params.yaml` | 改进版 TEB 参数（无后退、差速驱动、最短路径） |
| `src/wulina/param/straight_move_base_params.yaml` | 配套 move_base 参数（绕障 fallback 用） |
| `src/wulina/scripts/straight_line_nav.py` | 直线导航节点（rotate-then-translate 逻辑） |
| `docs/straight_line_navigation.md` | 本文档 |

> **所有原始文件均未修改。** 可与原来的多点导航方案共存，互不干扰。

---

## 三、设计原则与实现方案

### 3.1 只前进不后退

**参数层面（`straight_teb_params.yaml`）：**

```yaml
max_vel_x_backwards: 0.0          # 完全禁止后退（原值 0.15）
weight_kinematics_forward_drive: 100.0  # 强制前进优先（原值 1）
```

- `max_vel_x_backwards: 0.0` 从速度约束层面硬性禁止后退。  
- `weight_kinematics_forward_drive` 提高到 100，即使在约束允许的边界情况下
  也强烈偏好前向运动。

**节点层面（`straight_line_nav.py`）：**  
所有 `/cmd_vel` 发布仅在以下两种情形：
- Phase 1/3（旋转）：`linear.x = 0`，`angular.z = ±angular_speed`  
- Phase 2（前进）：`linear.x ≥ 0.04`，`angular.z = 0`  

代码中不存在 `linear.x < 0` 的发布路径。

---

### 3.2 转弯与前进不同时进行

**节点层面（`straight_line_nav.py`）实现了串行三阶段：**

```
Phase 1 → 原地旋转：使机器人朝向目标方向
           linear.x = 0 ; angular.z ≠ 0

Phase 2 → 直线前进：走到目标点 XY 位置
           linear.x > 0 ; angular.z = 0
           ↳ 若朝向偏差超过 heading_correct_threshold
             → 先停下，执行 Phase 1 重新对准，再继续前进

Phase 3 → 原地旋转：调整到目标点要求的最终偏航角
           linear.x = 0 ; angular.z ≠ 0
```

三个阶段串行执行，任意时刻线速度与角速度中只有一个非零。

---

### 3.3 无障碍物时规划直线路径

**参数层面（`straight_teb_params.yaml`）：**

```yaml
weight_shortest_path: 1.0            # 最小化路径长度（原值 0）
global_plan_overwrite_orientation: True  # TEB 跟随全局路径朝向（原值 False）
min_turning_radius: 0.0              # 差速底盘，无最小转弯半径约束
wheelbase: 0.0
```

全局规划器（`GlobalPlanner`，Dijkstra 算法）本身就给出地图上的最短路径，
TEB 在局部跟踪时开启 `weight_shortest_path` 后，会同时最小化路径长度，
从而更忠实地沿直线轨迹行驶。

**节点层面（`straight_line_nav.py`）：**  
在 Phase 2 中，导航节点直接沿"当前位置 → 目标 XY"的直线方向发送前进指令，
完全不经过 move_base 的局部规划，实现真正意义上的直线前进。

---

### 3.4 遇障自动切换 move_base 绕障

Phase 2 前进时订阅 `/scan`，若前方 ±30° 扫描锥内有距离 < `obstacle_dist` 的
激光束，则停止直线模式，自动调用 move_base 完成该段绕障导航，绕障完成后
继续使用直线模式前往下一个航点。

---

## 四、使用方法

### 4.1 启动

```bash
# 基本启动（默认参数）
roslaunch wulina straight_line_navigation.launch

# 指定地图并关闭交互模式（自动执行全部航点）
roslaunch wulina straight_line_navigation.launch \
  map_file:=/path/to/your_map.yaml \
  interactive:=false

# 循环巡逻 3 轮
roslaunch wulina straight_line_navigation.launch loop:=true loop_count:=3
```

### 4.2 与原方案共存

本方案与 `multi_point_navigation.launch` 不冲突；但两者都会启动 `move_base`
和硬件驱动，**不要同时运行两个 launch 文件**。

---

## 五、可调参数说明

所有参数均可通过 launch 文件命令行覆盖（`key:=value`），也可直接修改
launch 文件中的 `default` 值或对应 YAML 文件。

### 5.1 运动控制参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `linear_speed` | `0.10` m/s | Phase 2 直线前进速度上限。靠近目标时会自动减速（`dist × 0.5`，最低 0.04 m/s）。建议范围：0.06–0.20 m/s；过快可能使旋转阶段震荡。 |
| `angular_speed` | `0.30` rad/s | Phase 1/3 原地旋转角速度。建议范围：0.2–0.8 rad/s；太小会导致旋转时间过长，太大会超调。 |
| `yaw_tolerance` | `0.05` rad（≈3°） | 旋转到位容差。越小越精确，但旋转时间增加且可能在终止附近振荡。建议 0.03–0.10 rad。 |
| `xy_goal_tolerance` | `0.15` m | 到达目标点的 XY 距离容差。越小定位越精确，但可能因里程计累积误差导致无法到达。建议 0.10–0.25 m。 |
| `heading_correct_threshold` | `0.20` rad（≈11.5°） | Phase 2 前进中触发停下-重新对准的朝向偏差阈值。越小对齐越好但会频繁停下重新旋转；越大则路径更流畅但可能稍有弯曲。建议 0.15–0.35 rad。 |

### 5.2 障碍物检测参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `obstacle_dist` | `0.50` m | 前方障碍物触发距离。小于此距离的激光束视为障碍物，触发 move_base 绕障。根据机器人前端到激光安装点的距离适当调整，避免误触发。建议 0.30–0.80 m。 |
| `obstacle_angle` | `0.523` rad（≈30°） | 前方障碍检测半角（激光扫描帧中 \|angle\| ≤ 此值的光束参与检测）。越大检测范围越宽，越小越只检查正前方。建议 0.35–0.70 rad（20°–40°）。 |

### 5.3 TEB 参数（`straight_teb_params.yaml`）

> 以下参数在 move_base fallback 场景（绕障时）生效。正常直线前进不经过 TEB。

| 参数 | 本文件值 | 原始值 | 说明 |
|---|---|---|---|
| `max_vel_x_backwards` | `0.0` | `0.15` | 后退速度上限，设为 0 禁止后退 |
| `min_turning_radius` | `0.0` | `0.750` | 差速底盘无最小转弯半径约束 |
| `wheelbase` | `0.0` | `0.322` | 差速底盘轴距（阿克曼参数，设 0） |
| `weight_kinematics_forward_drive` | `100.0` | `1` | 前进优先权重，越高越不愿意后退 |
| `weight_shortest_path` | `1.0` | `0` | 路径长度优化权重，正值使 TEB 倾向于直线路径 |
| `global_plan_overwrite_orientation` | `True` | `False` | 让 TEB 跟随全局路径朝向，减少 S 形弯路 |

**调参建议：**
- 若绕障时机器人仍尝试倒退：进一步提高 `weight_kinematics_forward_drive`（如 200–500）。
- 若绕障路径仍弯曲过多：提高 `weight_shortest_path`（如 2.0–5.0）。
- 若 TEB 仍报"无法满足运动学约束"：确认 `min_turning_radius` 和 `wheelbase` 均为 0.0。

---

## 六、常见问题

**Q: 机器人前进中频繁停下来重新对准，行进很慢。**  
A: 提高 `heading_correct_threshold`（如改为 0.30–0.40 rad），或减小 `angular_speed`
   以降低旋转超调导致的重复纠偏。

**Q: 遇到障碍物后，move_base 绕障时仍然出现后退。**  
A: 这是 TEB fallback 行为，由 `straight_teb_params.yaml` 控制。
   确认 `max_vel_x_backwards: 0.0` 和 `min_turning_radius: 0.0` 已正确加载。
   可用 `rosparam get /move_base/TebLocalPlannerROS/max_vel_x_backwards` 验证。

**Q: 机器人绕了很大的弧才到目标点，没有走直线。**  
A: 检查 `min_turning_radius` 是否为 0，以及底盘是否真的是差速驱动。
   若为阿克曼底盘，则无法原地旋转，"先转后走直线"原则本身不适用。

**Q: 激光扫描方向与机器人前进方向不一致。**  
A: `obstacle_angle` 基于激光扫描帧（`laser` frame）的 angle=0 方向。
   若激光安装有偏转角，需相应调整 `obstacle_angle`，或在激光 TF 中修正安装偏角。
