# Environment Width Estimator 集成指南

## 概述

环境宽度实时估计器（Environment Width Estimator）是一个为有腿机器人在受限环境中导航而设计的模块。它使用侧向射线投射（Lateral Ray-Casting, LRC）方法实时估计走廊宽度，并基于环境宽度自适应调整轨迹优化参数。

## 功能特性

### 1. 侧向射线投射（Lateral Ray-Casting, LRC）
- **原理**：沿垂直于机器人前进方向发射 N 条平行射线
- **左右障碍物检测**：在 local_costmap 中找到左右最近的障碍物
- **宽度计算**：$W_t = d_{left} + d_{right}$

### 2. 噪声抑制
- **中值滤波**：对多条射线的测量结果取中值，抑制单帧噪声
- **EMA 平滑**：指数移动平均（Exponential Moving Average）滤波，平衡响应性和稳定性

### 3. 自适应约束切换
- **滞后态逻辑**：使用 Schmitt Trigger 逻辑防止模式振荡
- **离散约束切换**：Normal 模式 ↔ Narrow 模式
- **连续权重调整**：sigmoid 函数平滑调整成本函数权重

### 4. 有腿机器人特定成本函数
- **曲率平滑成本** ($J_\kappa$)：抑制振荡抖动
- **角速度连续性成本** ($J_{\dot{\omega}}$)：确保平滑的转向过渡

## 文件结构

```
teb_local_planner/
├── include/teb_local_planner/
│   └── environment_width_estimator.h          # 头文件
├── src/
│   └── environment_width_estimator.cpp        # 实现文件
│   └── teb_local_planner_ros.cpp              # 已集成到主插件
└── CMakeLists.txt                             # 需要添加源文件
```

## 集成步骤

### 步骤 1：编译配置

在 `CMakeLists.txt` 中添加新的源文件：

```cmake
# Find existing add_library 或 add_executable 指令
# 在其源文件列表中添加：
add_library(teb_local_planner
  src/teb_local_planner_ros.cpp
  src/timed_elastic_band.cpp
  src/environment_width_estimator.cpp    # 新增
  src/optimal_planner.cpp
  # ... 其他源文件 ...
)
```

### 步骤 2：ROS 2 参数配置

在你的启动文件或参数文件中配置环境宽度估计器参数：

```yaml
# 例如在 nav2_params.yaml 中添加：

teb_local_planner:
  ros__parameters:
    # ... 其他 TEB 参数 ...
    
    # 环境宽度估计器配置
    enable_width_estimation: true           # 启用/禁用估计器
    num_rays: 5                             # 射线数量（5-9）
    ray_spacing: 0.1                        # 射线间距 [m]
    ema_alpha: 0.3                          # EMA 平滑因子 (0.0-1.0)
    
    # 模式切换阈值
    width_threshold: 0.6                    # 走廊宽度阈值 [m]
    hysteresis_band: 0.1                    # 滞后带宽 [m]
    max_search_distance: 2.0                # 最大搜索距离 [m]
    
    # Narrow 模式约束
    narrow_max_vel_x: 0.2                   # 窄走廊最大线性速度 [m/s]
    narrow_max_vel_theta: 0.15              # 窄走廊最大角速度 [rad/s]
    narrow_min_obstacle_dist: 0.3           # 窄走廊最小障碍物距离 [m]
    
    # 曲率平滑成本
    enable_curvature_smoothing: true        # 启用曲率平滑
    weight_curvature_smoothing_normal: 0.1  # Normal 模式权重
    weight_curvature_smoothing_narrow: 0.5  # Narrow 模式权重
    
    # 角速度连续性成本
    enable_angular_smoothing: true          # 启用角速度平滑
    weight_angular_smoothing_normal: 0.3    # Normal 模式权重
    weight_angular_smoothing_narrow: 1.0    # Narrow 模式权重
```

### 步骤 3：编译和测试

```bash
cd ~/chapt7_ws
colcon build --packages-select teb_local_planner
source install/setup.bash
```

## 使用示例

### 基本使用（已自动集成）

环境宽度估计器在 TebLocalPlannerROS 的 `computeVelocityCommands()` 方法中自动调用。无需额外的代码修改。

### 可视化和调试

启用 ROS 日志查看估计的走廊宽度：

```bash
# 设置日志级别为 DEBUG
ros2 run teb_local_planner teb_local_planner_node --ros-args --log-level debug
```

输出示例：
```
[DEBUG] Corridor width: 0.850 m, State: Normal
[DEBUG] Corridor width: 0.520 m, State: Narrow
```

### 获取实时走廊宽度数据

可以在扩展 TebLocalPlannerROS 类后添加 ROS 发布者：

```cpp
// 在 teb_local_planner_ros.h 中添加
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corridor_width_pub_;

// 在 teb_local_planner_ros.cpp 的 initialize() 中添加
corridor_width_pub_ = node->create_publisher<std_msgs::msg::Float32>(
  "corridor_width", rclcpp::SystemDefaultsQoS());

// 在 computeVelocityCommands() 中发布
if (env_width_estimator_)
{
  std_msgs::msg::Float32 width_msg;
  width_msg.data = env_width_estimator_->getSmoothedWidth();
  corridor_width_pub_->publish(width_msg);
}
```

## 算法原理

### 射线投射算法

1. **初始化**：在机器人当前位置，计算垂直于前进方向的单位向量
   $$\mathbf{n}_t = \mathbf{v}_t^\perp$$

2. **射线铸造**：对于第 $i$ 条射线 ($i = 0, 1, ..., N-1$)
   - 起点：$\mathbf{p}_i = \mathbf{p}_t + i \cdot \delta d \cdot \mathbf{l}_t$（沿纵向分布）
   - 方向：$\pm \mathbf{n}_t$（左右两侧）

3. **障碍物检测**：沿射线逐步搜索
   $$d_{L/R,i} = \min\{d \mid \text{Costmap}(\mathbf{p}_i + d \cdot \mathbf{n}_t) = \text{Occupied}\}$$

4. **宽度估计**：使用中值滤波
   $$W_t = \text{median}(d_{L,1} + d_{R,1}, ..., d_{L,N} + d_{R,N})$$

5. **平滑滤波**：应用 EMA
   $$W_t^{smooth} = \alpha W_t^{raw} + (1-\alpha) W_{t-1}^{smooth}$$

### 自适应约束映射

- **Normal 模式** ($W_t > W_{th} + \delta/2$)：
  - $v_{max} = v_{max}^{base}$
  - $\omega_{max} = \omega_{max}^{base}$
  - $d_{min} = d_{min}^{base}$

- **Narrow 模式** ($W_t < W_{th} - \delta/2$)：
  - $v_{max} = v_{max}^{narrow}$ (更小)
  - $\omega_{max} = \omega_{max}^{narrow}$ (更小)
  - $d_{min} = d_{min}^{narrow}$ (更小)

## 参数调优指南

### 对于 Unitree Go1 机器人
| 参数 | 推荐值 | 范围 | 说明 |
|------|--------|------|------|
| `num_rays` | 5 | 5-9 | 机器人长度 ~0.5m，每条射线间距 0.1m 覆盖 0.4m |
| `ray_spacing` | 0.1 | 0.05-0.15 | 约为 costmap 分辨率的 2 倍 |
| `ema_alpha` | 0.3 | 0.1-0.5 | 更高值 = 更快响应，更低值 = 更平滑 |
| `width_threshold` | 0.6 | 0.55-0.65 | 机器人宽度(0.31m) + 腿部(0.18m) + 安全余量(0.11m) |
| `hysteresis_band` | 0.1 | 0.08-0.15 | 约为宽度测量噪声标准差的 2-3 倍 |

### 性能调试

1. **如果机器人在走廊中抖动**：
   - 增加 `ema_alpha` 或 `num_rays`
   - 增加 `weight_curvature_smoothing_narrow`

2. **如果机器人无法通过窄走廊**：
   - 降低 `width_threshold`
   - 增加 `narrow_min_obstacle_dist` 减小（允许更接近障碍物）
   - 检查 costmap 膨胀半径

3. **如果切换模式太频繁**：
   - 增加 `hysteresis_band`
   - 增加 `ema_alpha` 使宽度估计更平稳

## 计算复杂度

- **射线投射**：$O(N \cdot M)$，其中 $N$ 是射线数，$M$ 是搜索距离内的 costmap 单元数
- **中值滤波**：$O(N \log N)$
- **总体**：$<$ 1ms（在 Intel NUC i7 上，50Hz 控制循环）

## 故障排除

### 问题 1：走廊宽度始终为 0 或负数
**原因**：射线没有检测到任何障碍物或超出 costmap 范围
**解决方案**：
- 增加 `max_search_distance`
- 检查 costmap 是否正确初始化
- 验证机器人位置是否在 costmap 内

### 问题 2：自适应约束不生效
**原因**：`enable_width_estimation` 设置为 false 或估计器未正确初始化
**解决方案**：
- 检查日志中的初始化消息
- 验证参数是否正确加载

### 问题 3：机器人在模式之间快速切换
**原因**：走廊宽度在阈值附近波动
**解决方案**：
- 增加 `hysteresis_band` 值
- 增加 `ema_alpha` 平滑滤波

## 论文参考

详见 [paper_draft.tex](../paper_draft.tex) 中的相关章节：
- 第 III-B 小节：环境感知自适应约束
- 第 III-C 小节：有腿机器人特定成本函数

## 许可证

遵循 TEB Local Planner 的 BSD 许可证。
