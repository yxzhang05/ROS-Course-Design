# 调参指南：禁止小车后退 & 使用最短路径导航

本文档面向本课程设计的开发者，针对多点导航中**小车无故后退**以及**路径非最短**两个具体问题，
逐一分析根因并给出**需要修改的参数位置和推荐值**。

> **重要**：本文档只说明**如何改**，**不直接替换**任何参数文件。
> 所有参数均位于小车上 `catkin_roscar/src/start_roscar/param/` 目录，
> 请根据自己小车的实际硬件情况酌情调整数值。

---

## 目录

1. [param 目录文件一览](#1-param-目录文件一览)
2. [问题一：小车在空旷区域仍后退](#2-问题一小车在空旷区域仍后退)
   - [2.1 根因分析](#21-根因分析)
   - [2.2 需要修改的参数（teb_local_planner_params.yaml）](#22-需要修改的参数teb_local_planner_paramsyaml)
3. [问题二：路径非最短](#3-问题二路径非最短)
   - [3.1 根因分析](#31-根因分析)
   - [3.2 需要修改的参数（teb_local_planner_params.yaml）](#32-需要修改的参数teb_local_planner_paramsyaml)
   - [3.3 需要确认的参数（base_global_planner_param.yaml）](#33-需要确认的参数base_global_planner_paramyaml)
4. [推荐修改汇总表](#4-推荐修改汇总表)
5. [修改后的验证步骤](#5-修改后的验证步骤)
6. [诊断命令速查](#6-诊断命令速查)

---

## 1. param 目录文件一览

| 文件名 | 作用 |
|--------|------|
| `move_base_params.yaml` | move_base 全局设置：全局/局部规划器选择、频率、恢复行为开关 |
| `base_global_planner_param.yaml` | 全局规划器（GlobalPlanner）参数：算法选择、代价权重 |
| `teb_local_planner_params.yaml` | 局部规划器（TEB）参数：速度限制、运动学约束、优化权重 |
| `costmap_common_params.yaml` | 全局 & 局部代价地图公用参数：机器人外形、障碍层、膨胀层 |
| `global_costmap_params.yaml` | 全局代价地图专属参数：坐标系、更新频率、地图层插件列表 |
| `local_costmap_params.yaml` | 局部代价地图专属参数：滚动窗口大小、更新频率 |

---

## 2. 问题一：小车在空旷区域仍后退

### 2.1 根因分析

打开 `teb_local_planner_params.yaml`，可以看到以下几行与后退密切相关：

```yaml
max_vel_x_backwards: 0.15       # 当前：允许后退，最大速度 0.15 m/s
weight_kinematics_forward_drive: 1  # 当前：惩罚后退的权重极低（默认 1）
min_turning_radius: 0.750       # 当前：0.75 m 最小转弯半径（阿克曼车设置）
wheelbase: 0.322                # 当前：0.322 m 轴距（阿克曼车设置）
```

TEB 局部规划器在求解轨迹时会同时考虑运动学约束：

- `min_turning_radius: 0.750` 告诉 TEB 该小车是**阿克曼底盘**，原地无法转向，
  最小曲率半径为 0.75 m。  
  这意味着当全局路径要求小车在狭小空间内掉头时，TEB 会**主动规划"前进→后退"的倒车动作**
  来绕过转弯半径约束——即使周围完全没有障碍物，也会后退。
- `weight_kinematics_forward_drive: 1` 对"后退"的惩罚极低，TEB 几乎不会因为"尽量向前"
  的偏好而放弃倒车方案。
- `max_vel_x_backwards: 0.15` 允许小车以 0.15 m/s 的速度后退，不加任何限制。

**如果你的小车实际上是差速驱动（三轮差速 / 两轮差速），而非阿克曼小车**，
则 `min_turning_radius` 和 `wheelbase` 本应为 `0`，设为 0.75 m 完全是错误的运动学约束，
必然导致 TEB 虚构出一堆本不必要的倒车动作。

> **判断方法**：如果小车能原地旋转（两侧轮子一前一后同速转），则是差速驱动；
> 如果只有前轮转向、不能原地旋转，则是阿克曼底盘。
> 本仓库 `costmap_common_params.yaml` 的足迹顶点 `[[-0.09, -0.185], [-0.09, 0.185], [0.4, 0.185], [0.4, -0.185]]`
> 与 `teb_local_planner_params.yaml` 中注释里提到的 `senior_akm`（阿克曼小车）一致，
> 但实际场景中**小车能否原地旋转**需自行确认。

---

### 2.2 需要修改的参数（teb_local_planner_params.yaml）

#### 情形 A：小车是差速驱动（能原地旋转）——推荐方案

将阿克曼参数归零，同时禁止后退：

| 参数 | 当前值 | 建议值 | 说明 |
|------|--------|--------|------|
| `min_turning_radius` | `0.750` | `0.0` | 差速驱动最小转弯半径为 0，可原地旋转，无需倒车 |
| `wheelbase` | `0.322` | `0.0` | 差速驱动无"轴距"概念，设为 0 |
| `max_vel_x_backwards` | `0.15` | `0.0` | 完全禁止后退；若有特殊场景需要微小倒车，可设为 `0.02` |
| `weight_kinematics_forward_drive` | `1` | `100` | 大幅提高向前行驶的优化权重，使 TEB 强烈偏好前进方向 |

> **注意**：`max_vel_x_backwards: 0.0` 是禁止后退最直接的方式，
> 但若某些狭窄航点确实无法仅靠前进到达，TEB 将返回规划失败（ABORTED）。
> 可先设为 `0.02`（极慢后退）进行测试，确认无误后再改为 `0.0`。

#### 情形 B：小车是阿克曼底盘（不能原地旋转）

保留 `min_turning_radius` 和 `wheelbase`，但大幅提高前进偏好权重、降低后退速度上限：

| 参数 | 当前值 | 建议值 | 说明 |
|------|--------|--------|------|
| `max_vel_x_backwards` | `0.15` | `0.05` | 保留少量后退能力以应对真正的死角，但大幅限速 |
| `weight_kinematics_forward_drive` | `1` | `50` | 提高前进偏好，让 TEB 尽量找到不需要倒车的轨迹 |

> **提示**：阿克曼小车在 TEB 下要完全消除倒车比较困难，因为运动学约束本身就要求更大的转弯空间。
> 改善效果有限，建议优先确认小车底盘类型是否匹配。

---

## 3. 问题二：路径非最短

### 3.1 根因分析

本项目已在 `move_base_params.yaml` 中选用了 `GlobalPlanner` 作为全局规划器：

```yaml
base_global_planner: "global_planner/GlobalPlanner"
```

`base_global_planner_param.yaml` 中也已配置使用 Dijkstra 算法：

```yaml
GlobalPlanner:
  use_dijkstra: true   # Dijkstra 算法保证全局最优路径
  use_grid_path: false # false = 梯度下降法生成平滑路径（非严格最短格子路径）
```

**全局规划器本身已配置正确**：Dijkstra 保证找到在代价地图上代价最低（通常对应最短可行距离）的路径。

但在 TEB 局部规划器执行阶段，存在一个权重问题：

```yaml
# teb_local_planner_params.yaml（当前值）
weight_shortest_path: 0          # ← 为 0，TEB 完全不优化路径长度
weight_optimaltime: 1            # TEB 只优化时间
```

`weight_shortest_path: 0` 意味着 TEB 在跟随全局路径时，**完全不考虑路径长度**，
只考虑时间最优。在全局路径参考下，这通常不是关键问题，但如果全局路径本身较长
（如绕远路规避膨胀代价），TEB 不会尝试走捷径。

此外，`use_grid_path: false`（梯度下降法）生成的路径是平滑曲线，虽然视觉上不是
"格子最短路径"，但在大多数情况下比格子路径更短（因为可以走对角线）。

---

### 3.2 需要修改的参数（teb_local_planner_params.yaml）

| 参数 | 当前值 | 建议值 | 说明 |
|------|--------|--------|------|
| `weight_shortest_path` | `0` | `1` | 开启 TEB 的路径长度优化；从 1 开始，若路径还不够短可逐步加大到 5 |
| `max_global_plan_lookahead_dist` | `3.0` | `3.0`（无需改） | 预读全局路径的距离，3 m 适合室内小车 |

> **权衡提示**：`weight_shortest_path` 与 `weight_optimaltime` 存在竞争。
> 加大 `weight_shortest_path` 可能使小车稍慢（因为绕路更短但不一定更快），
> 需根据实际效果权衡。

---

### 3.3 需要确认的参数（base_global_planner_param.yaml）

以下参数当前值已较优，一般无需修改，但需了解其含义：

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `use_dijkstra` | `true` | **保持 true**。Dijkstra 保证在代价地图上找到最优路径；改为 false 则使用 A*（速度更快但不保证绝对最优） |
| `use_grid_path` | `false` | **保持 false**（梯度下降法，路径更平滑）。若改为 true 则严格沿格子边走，路径呈折线形、通常更长 |
| `neutral_cost` | `50` | 空闲区域的基础代价。值越高，规划器越倾向于走距离极短的路径（减少经过的格子数）；建议范围 `50~100` |
| `cost_factor` | `3.0` | 代价地图值乘以此系数后加入路径代价。增大此值会使规划器更强烈回避膨胀区域，路径会更远离墙壁 |

---

## 4. 推荐修改汇总表

### 4.1 针对差速驱动小车的完整修改清单

以下所有改动均在 `catkin_roscar/src/start_roscar/param/teb_local_planner_params.yaml` 中：

| 参数 | 原值 | 改为 | 目标效果 |
|------|------|------|----------|
| `min_turning_radius` | `0.750` | `0.0` | 消除阿克曼约束，允许原地转向 |
| `wheelbase` | `0.322` | `0.0` | 同上 |
| `max_vel_x_backwards` | `0.15` | `0.0` | 完全禁止后退 |
| `weight_kinematics_forward_drive` | `1` | `100` | TEB 强烈偏好前进方向 |
| `weight_shortest_path` | `0` | `1` | TEB 同时优化路径长度 |

在 `catkin_roscar/src/start_roscar/param/base_global_planner_param.yaml` 中无需修改（Dijkstra 已启用）。

### 4.2 针对阿克曼小车的修改清单

以下所有改动均在 `catkin_roscar/src/start_roscar/param/teb_local_planner_params.yaml` 中：

| 参数 | 原值 | 改为 | 目标效果 |
|------|------|------|----------|
| `max_vel_x_backwards` | `0.15` | `0.05` | 大幅降低后退速度上限 |
| `weight_kinematics_forward_drive` | `1` | `50` | 提高前进偏好 |
| `weight_shortest_path` | `0` | `1` | TEB 同时优化路径长度 |

---

## 5. 修改后的验证步骤

### 步骤 1：确认文件已保存并重新编译

由于参数文件是 YAML 配置（运行时加载），通常**不需要重新编译**，
但建议在修改后重新 source 一次环境：

```bash
cd ~/catkin_roscar
source devel/setup.bash
```

### 步骤 2：启动多点导航并观察

```bash
roslaunch wulina multi_point_navigation.launch
```

观察小车行为：
- **是否仍后退**：正常情况下，改动后小车在空旷区域应始终前进或原地旋转，不再后退。
- **路径是否合理**：在 RViz 中订阅 `/move_base/GlobalPlanner/plan` 话题，
  查看全局规划路径是否走了最短路线。

### 步骤 3：用 RViz 可视化验证

在 RViz 中添加以下显示项：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 全局规划路径（绿色线） |
| `/move_base/TebLocalPlannerROS/local_plan` | `nav_msgs/Path` | TEB 局部轨迹（黄色线） |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令（linear.x < 0 表示后退） |

### 步骤 4：实时监控速度命令

```bash
rostopic echo /cmd_vel
```

重点观察 `linear.x` 字段：
- `linear.x > 0`：小车前进（正常）
- `linear.x < 0`：小车后退（不期望出现）
- `linear.x = 0`，`angular.z ≠ 0`：小车原地旋转（可接受）

### 步骤 5：若改动后小车无法到达目标（ABORTED）

如果 `max_vel_x_backwards: 0.0` 导致小车无法规划到某些目标点（报 ABORTED），
可能的原因是：
1. 目标点附近空间太小，只靠前进无法到达 → 放宽 `max_vel_x_backwards` 为 `0.05`
2. 全局路径规划失败 → 检查目标点坐标是否在地图的可行区域内
3. `inflation_radius` 过大导致路径被阻断 → 在 `costmap_common_params.yaml` 中适当减小 `inflation_radius`

---

## 6. 诊断命令速查

```bash
# 实时查看速度命令（判断是否在后退）
rostopic echo /cmd_vel

# 查看 TEB 规划器诊断信息
rostopic echo /move_base/TebLocalPlannerROS/teb_feedback

# 查看全局规划路径
rostopic echo /move_base/GlobalPlanner/plan | head -50

# 查看 move_base 状态
rostopic echo /move_base/status

# 实时动态调整 TEB 参数（无需重启，用于快速实验）
rosrun rqt_reconfigure rqt_reconfigure
# 在弹出窗口中找到 /move_base/TebLocalPlannerROS，直接拖动滑块修改参数

# 查看当前生效的 TEB 参数列表
rosparam get /move_base/TebLocalPlannerROS

# 查看当前生效的全局规划器参数
rosparam get /move_base/GlobalPlanner
```

---

## 参考资料

- TEB 局部规划器官方文档：<http://wiki.ros.org/teb_local_planner>
- GlobalPlanner 官方文档：<http://wiki.ros.org/global_planner>
- move_base 官方文档：<http://wiki.ros.org/move_base>
- 本项目导航原理详解：[navigation_guide.md](navigation_guide.md)
