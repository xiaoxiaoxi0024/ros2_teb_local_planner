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

#ifndef EDGE_CURVATURE_SMOOTHING_H_
#define EDGE_CURVATURE_SMOOTHING_H_

#include <cmath>

#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/misc.h"

#include <Eigen/Core>

namespace teb_local_planner {

/**
 * @class EdgeCurvatureSmoothing
 * @brief 有腿机器人专用代价边 —— 曲率平滑成本 J_kappa
 *
 * 该边连接三个连续位姿 p_{i-1}, p_i, p_{i+1}，通过惩罚路径的离散弯曲能量
 * 来抑制轨迹中的急剧弯曲和振荡抖动，使轨迹在空间上更加平滑。
 *
 * 离散弯曲能量（bending energy）定义为：
 *   e = || p_{i-1} - 2 * p_i + p_{i+1} ||
 *
 * 这等价于路径位置的二阶离散差分的范数，物理含义是衡量路径在点 p_i 处
 * 的弯曲程度。对于有腿机器人，过大的曲率变化意味着需要快速调整足端落点，
 * 容易导致在窄走廊中步态不稳定。
 *
 * @see TebOptimalPlanner::AddEdgesCurvatureSmoothing
 * @remarks 务必先调用 setTebConfig() 设置配置
 */
class EdgeCurvatureSmoothing : public BaseTebMultiEdge<1, double>
{
public:
  /**
   * @brief 构造函数，初始化三顶点边
   */
  EdgeCurvatureSmoothing()
  {
    this->resize(3);  // 3 个位姿顶点: p_{i-1}, p_i, p_{i+1}
    this->setMeasurement(0.);
  }

  /**
   * @brief 计算代价函数
   *
   * 离散弯曲能量: e = || p_{i-1} - 2*p_i + p_{i+1} ||
   * 当三点共线时 e=0（无弯曲），弯曲越大 e 越大。
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeCurvatureSmoothing()");

    const VertexPose* pose0 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[2]);

    // 二阶离散差分向量: d2p = p0 - 2*p1 + p2
    Eigen::Vector2d d2p = pose0->position() - 2.0 * pose1->position() + pose2->position();

    // 代价 = 弯曲能量（二阶差分的范数）
    _error[0] = d2p.norm();

    TEB_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeCurvatureSmoothing::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace teb_local_planner

#endif /* EDGE_CURVATURE_SMOOTHING_H_ */
