/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Adaptive Trajectory Optimization for Legged Robots
 *********************************************************************/

#ifndef EDGE_ANGULAR_SMOOTHING_H_
#define EDGE_ANGULAR_SMOOTHING_H_

#include <cmath>

#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/misc.h"

#include <g2o/stuff/misc.h>  // g2o::normalize_theta

#include <Eigen/Core>

namespace teb_local_planner {

/**
 * @class EdgeAngularSmoothing
 * @brief 有腿机器人专用代价边 —— 角速度连续性成本 J_omega_dot
 *
 * 该边连接三个连续位姿 s_{i-1}, s_i, s_{i+1} 和两个时间差 dt_{i-1}, dt_i，
 * 通过惩罚相邻时间段之间角速度的变化（角加速度的软约束），确保转向过渡更加平滑。
 *
 * 角速度由相邻位姿的航向角差除以时间差得到：
 *   omega_i = normalize(theta_{i+1} - theta_i) / dt_i
 *
 * 角速度变化量（角加速度近似）定义为：
 *   e = omega_{i+1} - omega_i
 *     = (theta_{i+1} - theta_i)/dt_i - (theta_i - theta_{i-1})/dt_{i-1}
 *
 * 对于有腿机器人，角速度的突变会导致：
 * - 躯干产生额外的惯性力矩，影响步态稳定性
 * - 在窄走廊中需要频繁调整足端位置，增加滑倒风险
 * - 控制器跟踪误差增大，实际执行轨迹偏离规划轨迹
 *
 * 与已有的 EdgeAcceleration 的区别：
 * - EdgeAcceleration 对角加速度使用硬约束边界惩罚（penaltyBoundToInterval），
 *   只在超出 acc_lim_theta 时产生惩罚
 * - EdgeAngularSmoothing 使用二次型软惩罚，在整个值域上持续提供平滑梯度，
 *   使角速度变化全局最小化而非仅满足约束
 *
 * @see TebOptimalPlanner::AddEdgesAngularSmoothing
 * @remarks 务必先调用 setTebConfig() 设置配置
 */
class EdgeAngularSmoothing : public BaseTebMultiEdge<1, double>
{
public:
  /**
   * @brief 构造函数，初始化五顶点边
   */
  EdgeAngularSmoothing()
  {
    this->resize(5);  // 3 个位姿顶点 + 2 个时间差顶点
    this->setMeasurement(0.);
  }

  /**
   * @brief 计算代价函数
   *
   * omega_0 = normalize(theta_1 - theta_0) / dt_0
   * omega_1 = normalize(theta_2 - theta_1) / dt_1
   * e = omega_1 - omega_0  （角速度变化量）
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAngularSmoothing()");

    const VertexPose* pose0 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt0 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // 计算相邻位姿间的归一化航向角差
    const double angle_diff0 = g2o::normalize_theta(pose1->theta() - pose0->theta());
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());

    // 计算两段的角速度
    // 防止除零：使用极小值保护
    const double dt0_val = std::max(dt0->dt(), 1e-6);
    const double dt1_val = std::max(dt1->dt(), 1e-6);

    const double omega0 = angle_diff0 / dt0_val;
    const double omega1 = angle_diff1 / dt1_val;

    // 代价 = 角速度变化量（差值越大，代价越大）
    _error[0] = omega1 - omega0;

    TEB_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeAngularSmoothing::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace teb_local_planner

#endif /* EDGE_ANGULAR_SMOOTHING_H_ */
