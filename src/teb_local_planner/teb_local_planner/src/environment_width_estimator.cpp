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
 * Author: Adaptive Trajectory Optimization for Legged Robots
 *********************************************************************/

#include "teb_local_planner/environment_width_estimator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace teb_local_planner
{

EnvironmentWidthEstimator::EnvironmentWidthEstimator(int num_rays, double ray_spacing, double ema_alpha)
  : num_rays_(num_rays), 
    ray_spacing_(ray_spacing), 
    ema_alpha_(ema_alpha),
    raw_width_(0.0),
    smoothed_width_(0.0),
    current_state_(0),
    previous_state_(0)
{
  left_ray_distances_.resize(num_rays, 0.0);
  right_ray_distances_.resize(num_rays, 0.0);
}

double EnvironmentWidthEstimator::estimateCorridorWidth(
  double robot_x, double robot_y, double robot_theta,
  const nav2_costmap_2d::Costmap2D* costmap,
  double max_search_distance)
{
  if (!costmap) {
    return -1.0;
  }

  // Clear previous ray results
  std::fill(left_ray_distances_.begin(), left_ray_distances_.end(), 0.0);
  std::fill(right_ray_distances_.begin(), right_ray_distances_.end(), 0.0);

  // Compute perpendicular direction (lateral direction perpendicular to robot heading)
  // If robot heading is theta, perpendicular direction is theta + pi/2 (left side)
  double lateral_dir_x = -std::sin(robot_theta);  // perpendicular to heading (left)
  double lateral_dir_y = std::cos(robot_theta);

  // Longitudinal direction (along robot heading)
  double longi_dir_x = std::cos(robot_theta);
  double longi_dir_y = std::sin(robot_theta);

  // Set default max distance if not specified
  double search_distance = max_search_distance;
  if (search_distance <= 0.0) {
    search_distance = std::numeric_limits<double>::max();
  }

  // Cast N parallel rays perpendicular to heading
  for (int i = 0; i < num_rays_; ++i) {
    // Ray position along the robot's longitudinal axis
    // Center the rays around the robot position
    int center_offset = (num_rays_ - 1) / 2;
    int ray_index = i - center_offset;
    
    double ray_start_x = robot_x + ray_index * ray_spacing_ * longi_dir_x;
    double ray_start_y = robot_y + ray_index * ray_spacing_ * longi_dir_y;

    // Cast ray to the left
    left_ray_distances_[i] = castRayInCostmap(
      ray_start_x, ray_start_y,
      lateral_dir_x, lateral_dir_y,
      costmap, search_distance);

    // Cast ray to the right (opposite direction)
    right_ray_distances_[i] = castRayInCostmap(
      ray_start_x, ray_start_y,
      -lateral_dir_x, -lateral_dir_y,
      costmap, search_distance);
  }

  // Compute raw width as median of (left + right) clearances
  std::vector<double> widths;
  for (int i = 0; i < num_rays_; ++i) {
    widths.push_back(left_ray_distances_[i] + right_ray_distances_[i]);
  }

  raw_width_ = computeMedian(widths);

  // Apply EMA smoothing if enabled
  if (ema_alpha_ > 0.0) {
    if (width_history_.empty()) {
      smoothed_width_ = raw_width_;
    } else {
      smoothed_width_ = ema_alpha_ * raw_width_ + (1.0 - ema_alpha_) * smoothed_width_;
    }
    width_history_.push_back(raw_width_);
    if (width_history_.size() > HISTORY_SIZE) {
      width_history_.pop_front();
    }
  } else {
    smoothed_width_ = raw_width_;
  }

  return smoothed_width_;
}

double EnvironmentWidthEstimator::castRayInCostmap(
  double ray_start_x, double ray_start_y,
  double ray_direction_x, double ray_direction_y,
  const nav2_costmap_2d::Costmap2D* costmap,
  double max_distance) const
{
  // Normalize direction
  double dir_length = std::sqrt(ray_direction_x * ray_direction_x + ray_direction_y * ray_direction_y);
  if (dir_length < 1e-6) {
    return 0.0;
  }
  ray_direction_x /= dir_length;
  ray_direction_y /= dir_length;

  // Get costmap resolution and dimensions
  double resolution = costmap->getResolution();
  double step_size = resolution * 0.5;  // Use half resolution for finer ray stepping

  // Cast ray in steps until obstacle is found or max distance reached
  double distance = 0.0;
  while (distance < max_distance) {
    double check_x = ray_start_x + distance * ray_direction_x;
    double check_y = ray_start_y + distance * ray_direction_y;

    // Check if cell is occupied
    if (isCellOccupied(check_x, check_y, costmap)) {
      return distance;
    }

    distance += step_size;
  }

  return max_distance;
}

bool EnvironmentWidthEstimator::isCellOccupied(
  double x, double y, 
  const nav2_costmap_2d::Costmap2D* costmap) const
{
  // Check bounds
  if (x < costmap->getOriginX() || y < costmap->getOriginY() ||
      x > costmap->getOriginX() + costmap->getSizeInMetersX() ||
      y > costmap->getOriginY() + costmap->getSizeInMetersY()) {
    return true;  // Out of bounds treated as occupied
  }

  // Get cell indices
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return true;  // Invalid coordinates treated as occupied
  }

  // Check cell cost
  // In costmap_2d, cells with cost >= LETHAL_OBSTACLE (254) are obstacles
  // cells with cost = INSCRIBED_INFLATED_OBSTACLE (253) are inflation zones
  // cells with cost < INSCRIBED_INFLATED_OBSTACLE are free space
  unsigned char cost = costmap->getCost(mx, my);
  const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  
  return cost >= INSCRIBED_INFLATED_OBSTACLE;
}

double EnvironmentWidthEstimator::computeMedian(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());
  
  if (values.size() % 2 == 1) {
    return values[values.size() / 2];
  } else {
    return (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2.0;
  }
}

int EnvironmentWidthEstimator::getState(double width_threshold, double hysteresis_band) const
{
  if (smoothed_width_ < width_threshold - hysteresis_band / 2.0) {
    return 1;  // Narrow state
  } else if (smoothed_width_ > width_threshold + hysteresis_band / 2.0) {
    return 0;  // Normal state
  } else {
    return current_state_;  // Stay in current state (hysteresis)
  }
}

void EnvironmentWidthEstimator::reset()
{
  raw_width_ = 0.0;
  smoothed_width_ = 0.0;
  current_state_ = 0;
  previous_state_ = 0;
  width_history_.clear();
  std::fill(left_ray_distances_.begin(), left_ray_distances_.end(), 0.0);
  std::fill(right_ray_distances_.begin(), right_ray_distances_.end(), 0.0);
}

} // namespace teb_local_planner
