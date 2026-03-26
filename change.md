# 变更整理

## 3.7 调整

### 修改内容

1. `environment_width_estimator.cpp` / `environment_width_estimator.h`
   - 修复状态机逻辑。
   - `getState()` 现在具备真正的滞回记忆，不会在阈值边缘来回抖动。

2. `teb_local_planner_ros.cpp`
   - 去掉对 `weight_shortest_path` / `weight_prefer_rotdir` 的运行时错误改写。
   - 将正常模式下的 `min_obstacle_dist` 恢复为基线值，不再错误恢复为 `inflation_dist`。

3. `teb_config.h` / `teb_config.cpp`
   - 补齐 `env_width` 相关参数的声明和加载逻辑。
   - 解决此前参数大多只使用默认值、YAML 配置加载不可靠的问题。

4. `nav2_params.yaml`
   - 先将 `enable_width_estimation: false`。
   - 收紧 `oscillation_*` 相关参数。
   - 增加 `switching_blocking_period: 1.0`。
   - 关闭 `shrink_horizon_backup`。
   - 优先压制“冻结 / 卡住”问题。

### 根因判断

此前问题更像是以下三项因素叠加导致：

1. 宽度估计功能默认开启，但参数没有正确接入。
2. 正常模式错误恢复了障碍距离参数。
3. `homotopy` 与 `oscillation recovery` 在边界条件下反复拉扯。

## 3.12 改动

### 第四章 C. 腿部特定成本函数设计实现总结

根据论文第 IV-C 节，实现了两个有腿机器人专用代价函数，并以 g2o 优化边的形式集成到 TEB 轨迹优化框架中。

### 1. 曲率平滑成本 $J_\kappa$

- 文件：`edge_curvature_smoothing.h`
- 连接对象：三个连续位姿 $p_{i-1}, p_i, p_{i+1}$
- 目标：惩罚路径的离散弯曲能量（二阶位置差分）

公式：

$$
J_\kappa = \sum_i \left\lVert p_{i-1} - 2p_i + p_{i+1} \right\rVert^2
$$

作用：

- 抑制轨迹空间上的振荡和急弯。
- 使足端落点过渡更平稳。

### 2. 角速度连续性成本 $J_{\dot{\omega}}$

- 文件：`edge_angular_smoothing.h`
- 连接对象：三个连续位姿 $s_{i-1}, s_i, s_{i+1}$ 和两个时间差 $\Delta t_{i-1}, \Delta t_i$
- 目标：惩罚相邻轨迹段之间的角速度变化量

公式：

$$
\omega_i = \frac{\theta_{i+1} - \theta_i}{\Delta t_i}
$$

$$
J_{\dot{\omega}} = \sum_i \left( \omega_{i+1} - \omega_i \right)^2
$$

与现有 `EdgeAcceleration` 的区别：

- `EdgeAcceleration` 采用硬约束边界惩罚，仅在超限时起作用。
- `J_{omega_dot}` 采用二次型软惩罚，在整个值域上持续提供平滑梯度。

### 3. 自适应权重切换

在 `teb_local_planner_ros.cpp` 中，根据环境宽度估计器的状态动态调整权重：

| 模式 | `weight_curvature_smoothing` | `weight_angular_smoothing` |
| --- | --- | --- |
| Normal | 0.1 | 0.3 |
| Narrow | 0.5 | 1.0 |

### 修改文件清单

| 文件 | 改动内容 |
| --- | --- |
| `edge_curvature_smoothing.h` | 新增 $J_\kappa$ 对应的 g2o 代价边 |
| `edge_angular_smoothing.h` | 新增 $J_{\dot{\omega}}$ 对应的 g2o 代价边 |
| `teb_config.h` | 添加运行时活跃权重字段 |
| `optimal_planner.h` | 声明 `AddEdgesCurvatureSmoothing()` / `AddEdgesAngularSmoothing()` |
| `optimal_planner.cpp` | 实现上述方法，并在 `buildGraph()` 中调用 |
| `teb_local_planner_ros.cpp` | 初始化活跃权重，并在状态切换时动态更新 |

### 4. 其他问题根因

#### 倒着走的根因

1. `allow_init_with_backwards_motion: true`
   - 允许轨迹以倒退方式初始化。

2. `weight_kinematics_forward_drive: 2.0`
   - 惩罚倒退的权重偏低。

3. 缺少 `max_vel_x_backwards` 配置
   - 默认值 `0.2` 偏高。

4. `velocity_smoother` 的 `min_velocity`
   - 也允许 `-0.26` 的倒退速度。

#### 转圈的根因

1. `selection_cost_hysteresis: 5.0`
   - 值过高，导致规划器卡在次优同伦类。

2. `max_vel_theta: 1.0` 配合 `weight_max_vel_theta: 0.5`
   - 角速度约束权重偏低，旋转成本不足。

3. `homotopy` 规划切换
   - 可能反复在左绕 / 右绕方案之间切换。

## 3.19 改动

### 目标

收口“论文表述”和“实际代码实现”之间最影响说服力的四类不一致，原则是：

1. 代码对齐论文。
2. 论文只写真实实现。
3. 不再保留“论文是连续机制、代码是离散机制”这类答辩时一问就露出的断层。

### 本次修改内容

#### 1. 宽度估计公式与论文对齐

- 文件：`src/teb_local_planner/teb_local_planner/src/environment_width_estimator.cpp`
- 原实现：
  - 对每条射线先计算 `width_i = dL_i + dR_i`
  - 然后取 `median(width_i)`
- 现实现：
  - 分别计算 `median(dL)` 与 `median(dR)`
  - 最终宽度为 `median(dL) + median(dR)`

这一步把代码实现与论文 IV-B 中的宽度定义重新对齐。

#### 2. 滞回阈值与论文对齐

- 文件：`src/teb_local_planner/teb_local_planner/src/environment_width_estimator.cpp`
- 原实现：
  - 进入狭窄模式：`width_threshold - hysteresis_band / 2`
  - 退出狭窄模式：`width_threshold + hysteresis_band / 2`
- 现实现：
  - 进入狭窄模式：`width_threshold - hysteresis_band`
  - 退出狭窄模式：`width_threshold + hysteresis_band`

也就是现在代码真正对应论文中写的 `W_th ± \delta`，不再偷偷把 `\delta` 当成总带宽。

#### 3. 离散权重切换改为 sigmoid 连续调度

- 相关文件：
  - `src/teb_local_planner/teb_local_planner/src/teb_local_planner_ros.cpp`
  - `src/teb_local_planner/teb_local_planner/include/teb_local_planner/teb_config.h`
  - `src/teb_local_planner/teb_local_planner/src/teb_config.cpp`
  - `src/fishbot_navigation2/config/nav2_params.yaml`

- 原实现：
  - `Normal` / `Narrow` 两组离散权重硬切换
- 现实现：
  - `Normal` / `Narrow` 仍保留为状态机，用于约束切换
  - `weight_curvature_smoothing`
  - `weight_angular_smoothing`
  - 改为按走廊宽度做 sigmoid 连续插值

新增参数：

- `sigmoid_alpha`
  - 控制连续调度斜率

结论：

- 现在论文里“离散状态切换 + 连续权重调度”这两个层次在代码里都真实存在了。

#### 4. 补上动态 footprint 与 `Rmin` 切换

- 相关文件：
  - `src/teb_local_planner/teb_local_planner/src/teb_local_planner_ros.cpp`
  - `src/teb_local_planner/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h`
  - `src/teb_local_planner/teb_local_planner/include/teb_local_planner/teb_config.h`
  - `src/teb_local_planner/teb_local_planner/src/teb_config.cpp`
  - `src/fishbot_navigation2/config/nav2_params.yaml`

新增能力：

- 狭窄模式下动态切换 `min_turning_radius`
- 狭窄模式下可切换到独立的 `narrow_footprint_vertices`
- 正常模式恢复基线 footprint 与基线 `min_turning_radius`

新增参数：

- `narrow_min_turning_radius`
- `enable_dynamic_footprint`
- `narrow_footprint_vertices`
- `base_min_turning_radius`

说明：

- 这一步之前，论文里写了“动态 footprint + Rmin 切换”，但代码里并没有完整链路。
- 现在已经补成真实实现，不再是只改速度和障碍距离的“半套自适应”。

#### 5. 修正基线参数恢复逻辑

- 相关文件：
  - `src/teb_local_planner/teb_local_planner/include/teb_local_planner/teb_config.h`
  - `src/teb_local_planner/teb_local_planner/src/teb_config.cpp`

修复内容：

- 补齐 `base_max_vel_x`
- `base_max_vel_theta`
- `base_min_obstacle_dist`
- `base_min_turning_radius`

确保从狭窄模式退回正常模式时，恢复的是 YAML / 运行配置中的真实基线值，而不是构造默认值。

这个修复很重要，否则代码即使“有切换”，也可能在切回来时把参数恢复错。

#### 6. 论文公式同步回写

- 文件：`paper_draft.tex`

修改内容：

- 将权重公式从“趋近 `w_max` 的单端 sigmoid”
- 改为“在 `w^{Narrow}` 与 `w^{Normal}` 之间插值的双端 sigmoid”

即：

$$
w_{\kappa}(W_t) = w_{\kappa}^{\text{Narrow}} + \frac{w_{\kappa}^{\text{Normal}} - w_{\kappa}^{\text{Narrow}}}{1 + e^{-\alpha(W_t - W_{th})}}
$$

这样论文内部也和现在的代码端点设置一致，不再出现“前文写两组端点、后文公式却像是从 0 长到最大值”的自相矛盾。

### 修改文件清单

| 文件 | 改动内容 |
| --- | --- |
| `environment_width_estimator.cpp` | 宽度估计公式改为 `median(dL)+median(dR)`，滞回阈值改为 `± hysteresis_band` |
| `teb_local_planner_ros.cpp` | 统一环境自适应入口，新增 sigmoid 权重调度、动态 footprint、动态 `Rmin` 切换 |
| `teb_local_planner_ros.h` | 补充环境自适应相关函数声明与基线/窄模式 footprint 成员 |
| `teb_config.h` | 新增 `narrow_min_turning_radius`、`enable_dynamic_footprint`、`narrow_footprint_vertices`、`sigmoid_alpha`、`base_min_turning_radius` |
| `teb_config.cpp` | 补齐上述参数的声明、加载与动态参数回调 |
| `nav2_params.yaml` | 新增狭窄模式 `Rmin`、动态 footprint、sigmoid 调度参数 |
| `paper_draft.tex` | 将权重公式与文字说明改成与代码一致的连续插值形式 |

### 验证结果

执行：

```bash
colcon build --packages-select teb_local_planner fishbot_navigation2 --event-handlers console_direct+
```

结果：

- 编译通过。
- `teb_local_planner` 与 `fishbot_navigation2` 均完成构建。

### 当前状态说明

代码层面，这一版已经把以下四个核心矛盾收口：

1. 宽度估计公式不一致。
2. 滞回阈值定义不一致。
3. 连续权重调度与离散切换不一致。
4. 论文写了动态 footprint / `Rmin`，但代码未真实实现。

但配置层面当前 `nav2_params.yaml` 里仍然保持：

- `enable_width_estimation: false`
- `enable_curvature_smoothing: false`
- `enable_angular_smoothing: false`
- `enable_dynamic_footprint: false`

也就是说：

- 代码能力已经补齐。
- 论文与实现逻辑已经可以对齐。
- 但如果要让运行实验也完全进入论文模式，下一步还需要把实验配置参数切到正式版。

## 3.25改动

### 主要内容

本次工作的重点是为四足机器人导航测试生成了 3 个可直接用于 Gazebo 的 SLAM 仿真场景，并接入 `fishbot_description` 的仿真启动链路。

### 新增 3 个 Gazebo SLAM 场景

新增 world 文件：

- `src/fishbot_description/world/slam_narrow_corridor.world`
- `src/fishbot_description/world/slam_s_curve_corridor.world`
- `src/fishbot_description/world/slam_l_turn_corridor.world`

对应场景如下：

1. 狭窄直走廊
   - 尺寸按 8m × 0.55m 构建，墙高 2.4m。
   - 场景中加入入口 AprilTag、矮柜、纸箱、地砖、白墙和顶灯。
   - 用于验证超窄通道下的 SLAM 与导航能力。

2. S 型弯路
   - 总长 12m，通道宽 0.8m，包含两个平滑转向。
   - 场景中加入弯道 AprilTag、杂物、立柱和门等视觉特征。
   - 用于验证连续弯道中的定位稳定性与路径跟踪能力。

3. 直角 L 型路
   - 通道宽 0.7m，两段各长 4m，内角半径 0.3m。
   - 场景中加入内角 AprilTag、垃圾桶、画框、踢脚线和开关面板。
   - 用于验证拐角转弯、视角突变和局部重定位表现。

### 场景设计特点

- 3 个场景都按论文要求控制了走廊宽度，分别为 0.55m、0.8m、0.7m。
- 每个场景都加入了较丰富的室内视觉特征，避免环境过空、纹理过少，提升 SLAM 可观测性。
- 场景风格尽量贴近真实室内环境，便于后续用于四足机器人导航实验复现。

### 启动链路配套修改

为了让新增场景能够直接通过现有 launch 使用，同时减少 Gazebo 串场问题，对仿真启动链路做了配套整理：

- 在 `src/fishbot_description/launch/gazebo_sim.launch.py` 中加入 `scene:=narrow|s_curve|l_turn` 场景选择参数。
- 保留 `world:=...` 直接指定 world 文件的方式。
- 增加独立 `GAZEBO_MASTER_URI`，避免多个 Gazebo 实例共用默认 master。
- 调整 GUI 启动方式，使 `gzserver` 与 `gzclient` 解耦，降低启动后显示错误场景的概率。

### 验证情况

已完成的验证包括：

- 3 个 `.world` 文件均能正常解析。
- `fishbot_description` 可正常编译安装。
- 新增场景可通过 `gazebo_sim.launch.py` 启动。
- `scene:=narrow` 与 `scene:=s_curve` 启动时，Gazebo master 对应的 world 名分别正确进入：
  - `slam_narrow_corridor`
  - `slam_s_curve_corridor`

### 说明

当前这 3 个 world 已经可以作为窄通道、弯道、转角三类典型室内导航测试场景使用。  
如果后续将机器人模型从当前 `fishbot` 替换为云深处 `lite3`，这些场景文件本身可以继续复用。
