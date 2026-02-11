#ifndef ENVIRONMENT_WIDTH_ESTIMATOR_H_
#define ENVIRONMENT_WIDTH_ESTIMATOR_H_

#include <Eigen/Core>
#include <vector>
#include <deque>
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace teb_local_planner
{

/**
 * @class EnvironmentWidthEstimator
 * @brief 基于侧向射线投射（Lateral Ray-Casting, LRC）方法的实时环境宽度估计器
 * 
 * 该估计器通过以下步骤计算机器人当前位置的可通行走廊宽度：
 * 1. 发射 N 条平行射线，方向垂直于机器人前进方向
 * 2. 检测射线左右两侧最近的障碍物
 * 3. 将左右两侧的无障碍距离求和，得到有效走廊宽度
 * 4. 应用中值滤波和/或指数移动平均（EMA）平滑，抑制测量噪声
 * 
 * 该方法针对实时性能优化，时间复杂度为 O(N)（N 为射线数量）。
 */
class EnvironmentWidthEstimator
{
public:
  /**
   * @brief 构造函数
   * @param num_rays 垂直于前进方向发射的射线数量（典型值 5-9 条）
   * @param ray_spacing 机器人纵轴方向上相邻射线的间距 [米]
   * @param ema_alpha 指数移动平均（EMA）平滑因子（0.0-1.0），0 表示禁用 EMA
   */
  EnvironmentWidthEstimator(int num_rays = 5, double ray_spacing = 0.1, double ema_alpha = 0.3);

  /**
   * @brief 析构函数（默认实现）
   */
  ~EnvironmentWidthEstimator() = default;

  /**
   * @brief 估计机器人当前位置的走廊宽度
   * 
   * @param robot_x 机器人当前 X 坐标（代价地图坐标系）
   * @param robot_y 机器人当前 Y 坐标（代价地图坐标系）
   * @param robot_theta 机器人当前航向角（弧度）
   * @param costmap nav2_costmap_2d::Costmap2D 对象的指针
   * @param max_search_distance 射线的最大搜索距离（0 表示无限制）
   * 
   * @return 估计的有效走廊宽度 [米]；若估计失败，返回 -1.0
   */
  double estimateCorridorWidth(
    double robot_x, double robot_y, double robot_theta,
    const nav2_costmap_2d::Costmap2D* costmap,
    double max_search_distance = 0.0);

  /**
   * @brief 获取经过平滑处理的走廊宽度估计值
   * @return 当前平滑后的宽度估计值 [米]
   */
  double getSmoothedWidth() const { return smoothed_width_; }

  /**
   * @brief 获取上一次评估的原始（未平滑）宽度估计值
   * @return 原始宽度估计值 [米]
   */
  double getRawWidth() const { return raw_width_; }

  /**
   * @brief 获取详细的射线投射结果（用于可视化/调试）
   * 
   * @param[out] left_distances 每条射线左侧的无障碍距离向量
   * @param[out] right_distances 每条射线右侧的无障碍距离向量
   */
  void getRaycastResults(std::vector<double>& left_distances, std::vector<double>& right_distances) const
  {
    left_distances = left_ray_distances_;
    right_distances = right_ray_distances_;
  }

  /**
   * @brief 基于滞回阈值切换逻辑，获取当前环境状态（正常/狭窄）
   * 
   * @param width_threshold 切换状态的宽度阈值 [米]
   * @param hysteresis_band 滞回带宽度 [米]
   * @return 0 表示正常状态，1 表示狭窄状态
   */
  int getState(double width_threshold, double hysteresis_band) const;

  /**
   * @brief 重置所有滤波器（清空历史数据）
   */
  void reset();

  /**
   * @brief 设置 EMA 平滑因子
   * @param alpha EMA 因子（取值范围 0.0-1.0）
   */
  void setEMAAlpha(double alpha)
  {
    if (alpha >= 0.0 && alpha <= 1.0)
      ema_alpha_ = alpha;
  }

  /**
   * @brief 设置下一次估计使用的射线数量
   * @param num_rays 射线数量（需大于 0）
   */
  void setNumRays(int num_rays)
  {
    if (num_rays > 0)
      num_rays_ = num_rays;
  }

private:
  /**
   * @brief 沿指定方向发射单条射线，查找到第一个障碍物的距离
   * 
   * @param ray_start_x 射线起点 X 坐标
   * @param ray_start_y 射线起点 Y 坐标
   * @param ray_direction_x 方向余弦（单位方向向量的 X 分量）
   * @param ray_direction_y 方向余弦（单位方向向量的 Y 分量）
   * @param costmap 代价地图指针
   * @param max_distance 最大搜索距离（0 = 无限制）
   * 
   * @return 到障碍物的距离 [米]；若未找到障碍物，返回 max_distance
   */
  double castRayInCostmap(
    double ray_start_x, double ray_start_y,
    double ray_direction_x, double ray_direction_y,
    const nav2_costmap_2d::Costmap2D* costmap,
    double max_distance) const;

  /**
   * @brief 计算向量的中值
   * @param values 输入向量
   * @return 中值
   */
  static double computeMedian(std::vector<double> values);

  /**
   * @brief 检查代价地图的某个栅格是否被占用
   * @param x 栅格 X 坐标（世界坐标系）
   * @param y 栅格 Y 坐标（世界坐标系）
   * @param costmap 代价地图指针
   * @return 若栅格被占用或超出边界，返回 true；否则返回 false
   */
  bool isCellOccupied(double x, double y, const nav2_costmap_2d::Costmap2D* costmap) const;

private:
  // 配置参数
  int num_rays_;                    //!< 发射的平行射线数量
  double ray_spacing_;              //!< 纵轴方向上射线的间距 [米]
  double ema_alpha_;                //!< EMA 平滑因子（0.0 表示禁用平滑）

  // 状态变量
  double raw_width_;                //!< 上一次估计的原始（未滤波）宽度
  double smoothed_width_;           //!< 平滑后的宽度（中值滤波 + EMA 后）
  int current_state_;               //!< 当前状态（0=正常，1=狭窄）
  int previous_state_;              //!< 上一状态（用于滞回逻辑）

  // 滤波用历史数据
  std::vector<double> left_ray_distances_;   //!< 左侧每条射线的无障碍距离
  std::vector<double> right_ray_distances_;  //!< 右侧每条射线的无障碍距离
  std::deque<double> width_history_;         //!< 宽度历史缓冲区（用于滤波）
  static constexpr int HISTORY_SIZE = 3;    //!< 滤波用历史缓冲区大小
};

// 定义智能指针类型别名，简化使用
using EnvironmentWidthEstimatorPtr = std::shared_ptr<EnvironmentWidthEstimator>;

} // namespace teb_local_planner

#endif // ENVIRONMENT_WIDTH_ESTIMATOR_H_