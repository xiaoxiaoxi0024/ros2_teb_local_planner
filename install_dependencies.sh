#!/bin/bash

# chapt7_ws 工作空间一键依赖安装脚本
# 适用于 ROS2 Humble

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否为 root 用户
check_root() {
    if [ "$EUID" -eq 0 ]; then 
        print_error "请不要使用 root 用户运行此脚本"
        exit 1
    fi
}

# 检查操作系统
check_os() {
    if [ ! -f /etc/os-release ]; then
        print_error "无法检测操作系统版本"
        exit 1
    fi
    
    . /etc/os-release
    print_info "检测到操作系统: $ID $VERSION_ID"
    
    if [ "$ID" != "ubuntu" ]; then
        print_warn "此脚本主要针对 Ubuntu 系统，其他系统可能需要手动调整"
    fi
}

# 检查 ROS2 是否已安装
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        print_warn "未检测到 ROS2 环境变量，将检查 /opt/ros/humble"
        if [ ! -d "/opt/ros/humble" ]; then
            print_error "未找到 ROS2 Humble 安装，请先安装 ROS2 Humble"
            print_info "安装命令参考:"
            echo "  sudo apt update"
            echo "  sudo apt install -y software-properties-common"
            echo "  sudo add-apt-repository universe"
            echo "  sudo apt update && sudo apt install -y curl gnupg lsb-release"
            echo "  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
            echo "  sudo sh -c 'echo \"deb [arch=\$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'"
            echo "  sudo apt update"
            echo "  sudo apt install -y ros-humble-desktop"
            exit 1
        else
            print_info "找到 ROS2 Humble 安装，但环境变量未设置"
            print_info "请运行: source /opt/ros/humble/setup.bash"
        fi
    else
        print_info "检测到 ROS_DISTRO: $ROS_DISTRO"
        if [ "$ROS_DISTRO" != "humble" ]; then
            print_warn "当前 ROS_DISTRO 为 $ROS_DISTRO，但工作空间可能需要 Humble"
        fi
    fi
}

# 初始化 rosdep
init_rosdep() {
    print_info "初始化 rosdep..."
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        print_info "首次运行，需要初始化 rosdep (需要 sudo 权限)..."
        sudo rosdep init || print_warn "rosdep 可能已经初始化"
    else
        print_info "rosdep 已初始化"
    fi
    
    print_info "更新 rosdep 数据库..."
    rosdep update || print_warn "rosdep update 失败，继续执行..."
}

# 安装系统依赖
install_system_dependencies() {
    print_info "安装系统依赖包..."
    
    # 更新包列表
    sudo apt update
    
    # 基础构建工具
    sudo apt install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-apt
    
    # libg2o (TEB planner 需要)
    print_info "安装 libg2o..."
    sudo apt install -y \
        libg2o-dev \
        libsuitesparse-dev \
        libeigen3-dev
    
    # OpenCV (cv_bridge 需要)
    print_info "安装 OpenCV 相关包..."
    sudo apt install -y \
        libopencv-dev \
        python3-opencv
    
    # 其他系统库
    sudo apt install -y \
        libboost-all-dev \
        libyaml-cpp-dev \
        libflann-dev \
        libpcl-dev
    
    print_info "系统依赖安装完成"
}

# 安装 ROS2 包依赖
install_ros2_dependencies() {
    print_info "安装 ROS2 包依赖..."
    
    # 确保 ROS2 环境已加载
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
            print_info "已加载 ROS2 Humble 环境"
        fi
    fi
    
    # 安装 Navigation2 相关包
    sudo apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-nav2-common \
        ros-$ROS_DISTRO-nav2-core \
        ros-$ROS_DISTRO-nav2-costmap-2d \
        ros-$ROS_DISTRO-nav2-msgs \
        ros-$ROS_DISTRO-nav2-util \
        ros-$ROS_DISTRO-nav2-simple-commander \
        ros-$ROS_DISTRO-dwb-core \
        ros-$ROS_DISTRO-dwb-critics \
        ros-$ROS_DISTRO-dwb-plugins \
        ros-$ROS_DISTRO-dwb-msgs \
        ros-$ROS_DISTRO-amcl \
        ros-$ROS_DISTRO-map-server \
        ros-$ROS_DISTRO-lifecycle-manager \
        ros-$ROS_DISTRO-nav2-lifecycle-manager \
        ros-$ROS_DISTRO-nav2-planner \
        ros-$ROS_DISTRO-nav2-controller \
        ros-$ROS_DISTRO-nav2-recoveries \
        ros-$ROS_DISTRO-nav2-behavior-tree \
        ros-$ROS_DISTRO-nav2-bt-navigator \
        ros-$ROS_DISTRO-nav2-waypoint-follower \
        ros-$ROS_DISTRO-nav2-velocity-smoother \
        ros-$ROS_DISTRO-nav2-rotation-smoother
    
    # ROS2 核心包
    sudo apt install -y \
        ros-$ROS_DISTRO-rclpy \
        ros-$ROS_DISTRO-rclcpp \
        ros-$ROS_DISTRO-rclcpp-action \
        ros-$ROS_DISTRO-rclcpp-lifecycle \
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-std-msgs \
        ros-$ROS_DISTRO-builtin-interfaces \
        ros-$ROS_DISTRO-visualization-msgs \
        ros-$ROS_DISTRO-tf2 \
        ros-$ROS_DISTRO-tf2-eigen \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-tf2-geometry-msgs \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-class-loader \
        ros-$ROS_DISTRO-pluginlib \
        ros-$ROS_DISTRO-rosidl-default-generators \
        ros-$ROS_DISTRO-rosidl-default-runtime
    
    # 测试和开发工具
    sudo apt install -y \
        ros-$ROS_DISTRO-ament-cmake \
        ros-$ROS_DISTRO-ament-lint \
        ros-$ROS_DISTRO-ament-lint-auto \
        ros-$ROS_DISTRO-ament-lint-common \
        ros-$ROS_DISTRO-ament-cmake-gtest \
        ros-$ROS_DISTRO-ament-copyright \
        ros-$ROS_DISTRO-ament-flake8 \
        ros-$ROS_DISTRO-ament-pep257 \
        python3-pytest
    
    print_info "ROS2 包依赖安装完成"
}

# 使用 rosdep 安装工作空间依赖
install_workspace_dependencies() {
    print_info "使用 rosdep 安装工作空间依赖..."
    
    # 获取脚本所在目录（工作空间根目录）
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    cd "$SCRIPT_DIR"
    
    # 确保在正确的目录
    if [ ! -d "src" ]; then
        print_error "未找到 src 目录，请确保在工作空间根目录运行此脚本"
        exit 1
    fi
    
    # 使用 rosdep 安装依赖
    print_info "正在解析并安装工作空间依赖（需要 sudo 权限）..."
    rosdep install --from-paths src --ignore-src -r -y || {
        print_warn "rosdep 安装过程中出现一些警告，但可能不影响编译"
    }
    
    print_info "工作空间依赖安装完成"
}

# 安装 Python 依赖
install_python_dependencies() {
    print_info "安装 Python 依赖..."
    
    pip3 install --user \
        setuptools \
        wheel \
        numpy \
        transforms3d || print_warn "部分 Python 包安装失败"
    
    print_info "Python 依赖安装完成"
}

# 验证安装
verify_installation() {
    print_info "验证关键依赖..."
    
    # 检查 colcon
    if command -v colcon &> /dev/null; then
        print_info "✓ colcon 已安装"
    else
        print_error "✗ colcon 未找到"
    fi
    
    # 检查 rosdep
    if command -v rosdep &> /dev/null; then
        print_info "✓ rosdep 已安装"
    else
        print_error "✗ rosdep 未找到"
    fi
    
    # 检查 ROS2
    if [ -n "$ROS_DISTRO" ]; then
        print_info "✓ ROS2 环境已设置 (ROS_DISTRO=$ROS_DISTRO)"
    else
        print_warn "⚠ ROS2 环境变量未设置，请运行: source /opt/ros/humble/setup.bash"
    fi
    
    # 检查 libg2o
    if pkg-config --exists g2o; then
        print_info "✓ libg2o 已安装"
    else
        print_warn "⚠ libg2o 可能未正确安装"
    fi
}

# 主函数
main() {
    print_info "=========================================="
    print_info "chapt7_ws 工作空间依赖安装脚本"
    print_info "=========================================="
    echo ""
    
    check_root
    check_os
    check_ros2
    
    echo ""
    print_info "开始安装依赖..."
    echo ""
    
    # 初始化 rosdep
    init_rosdep
    
    # 安装系统依赖
    install_system_dependencies
    
    # 安装 ROS2 包依赖
    install_ros2_dependencies
    
    # 安装工作空间依赖
    install_workspace_dependencies
    
    # 安装 Python 依赖
    install_python_dependencies
    
    echo ""
    print_info "=========================================="
    print_info "依赖安装完成！"
    print_info "=========================================="
    echo ""
    
    # 验证安装
    verify_installation
    
    echo ""
    print_info "下一步操作："
    echo "  1. 确保 ROS2 环境已加载:"
    echo "     source /opt/ros/humble/setup.bash"
    echo ""
    echo "  2. 编译工作空间:"
    echo "     cd $(pwd)"
    echo "     colcon build"
    echo ""
    echo "  3. 加载工作空间环境:"
    echo "     source install/setup.bash"
    echo ""
}

# 运行主函数
main

