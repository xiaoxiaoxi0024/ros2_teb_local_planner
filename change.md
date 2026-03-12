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
