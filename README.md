# ros2-teb_local_planner/fishros_ros2bookcode_chapter7
Teb_local_planner 包将一个插件实现到2D 导航堆栈的 base_local_planner。基本的方法称为时间弹性带局部优化机器人的轨迹执行时间，从障碍物分离和遵守运动学约束在运行时的轨迹。从效果上看Teb_local_planner比DWB更好，对全局路径紧密跟进，不会存在像DWB规划期那样转弯时会偏离全局路径较多。

但是由于TEB是在ros1时期发布的，且其维护者已离开学术界，转而从事其他工作，无法继续维护它。因此，TEB目前基本无人维护，导致ros2没有可以直接使用apt安装的debian包，所以需要使用源代码进行编译。


开发环境

操作系统：Ubuntu 22.04

ROS版本：ROS2 humble

本文件内容在**鱼香ros** chapter7中的导航基础上做了修改，将其局部规划算法DWB改为TEB

其中TEB规划器在**Christoph Rösmann**发布的teb_local_planner基础上做了部分修改


## 1、编译和安装


### 1.1已有仿真和导航规划器，需要在本地替换局部规划器为TEB
**直接打开已有工作空间**

```bash
cd ~/yourspace_ws/src
git clone https://github.com/xiaoxiaoxi0024/ros2_teb_local_planner/tree/1ea9d382072fe7a290c78393c471db438e675a27/src/costmap_converter
git clone https://github.com/xiaoxiaoxi0024/ros2_teb_local_planner/tree/1ea9d382072fe7a290c78393c471db438e675a27/src/teb_local_planner
```

**安装依赖**
```bash
rosdep install -i --from-path src --rosdistro humble -y  #cd ~/yourspace_ws/
```

**配置teb规划器**
打开teb_local_planner/teb_local_planner/params/teb_params.yaml文件，将里面的内容修改到自己的导航配置中（主要修改controller_serve部分）


### 1.2本地没有仿真和导航规划器

**创建新的工作空间**
```bash
mkdir -p  ~/teb_ws/
cd ~/teb_ws/
git clone https://github.com/xiaoxiaoxi0024/ros2_teb_local_planner.git
```

**安装依赖**
```bash
cd ~/teb_ws/
chmod +x install_dependencies.sh
bash ./install_dependencies.sh
```

**运行仿真**
```bash
cd ~/teb_ws/
colcon build
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
```
打开另一个终端
```bash
cd ~/teb_ws/
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

