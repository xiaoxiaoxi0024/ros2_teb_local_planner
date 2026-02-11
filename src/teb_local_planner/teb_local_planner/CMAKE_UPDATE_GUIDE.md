# CMakeLists.txt 更新指南

## 添加环境宽度估计器源文件

在你的 `teb_local_planner` 的 `CMakeLists.txt` 中找到包含源文件的 `add_library()` 或 `target_sources()` 命令。

### 示例位置

通常在 `src/CMakeLists.txt` 或根目录 `CMakeLists.txt` 中：

```cmake
add_library(teb_local_planner
  src/teb_local_planner_ros.cpp
  src/timed_elastic_band.cpp
  src/optimal_planner.cpp
  src/homotopy_class_planner.cpp
  src/obstacles.cpp
  src/recovery_behaviors.cpp
  src/teb_config.cpp
  src/visualization.cpp
  src/graph_search.cpp
  # 添加以下行：
  src/environment_width_estimator.cpp
)
```

或者使用 `target_sources()`：

```cmake
target_sources(teb_local_planner PRIVATE
  src/environment_width_estimator.cpp
)
```

## 编译验证

完成上述修改后，进行编译测试：

```bash
cd ~/chapt7_ws
colcon build --packages-select teb_local_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 可能的编译错误及解决方案

### 错误 1：找不到 "nav2_costmap_2d/costmap_2d.hpp"
**解决方案**：确保 `package.xml` 中已添加依赖项：
```xml
<depend>nav2_costmap_2d</depend>
```

### 错误 2：环境宽度估计器符号未定义
**解决方案**：
1. 确认 `src/environment_width_estimator.cpp` 已添加到 CMakeLists.txt
2. 清除构建目录后重新编译：
   ```bash
   rm -rf build install
   colcon build --packages-select teb_local_planner
   ```

### 错误 3：链接错误
**解决方案**：确保所有必要的库已链接到 `target_link_libraries()` 中：
```cmake
target_link_libraries(teb_local_planner
  PUBLIC
    nav2_costmap_2d::nav2_costmap_2d
    tf2
    rclcpp
)
```
