/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  if (FLAGS_use_st_drivable_boundary) {//false
    return false;
  }
  for (const auto* boundary : boundaries) {//遍历障碍物的stboundary
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
  }
  return false;
}
}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const std::vector<const Obstacle*>& obstacles,
    const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),
      gridded_path_time_graph_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) 
     //dp_st_cost_构造函数用来初始化cost，并将处于keep_clear区域的障碍物的
     //start_s, end_s放入keep_clear_range_中，并对其排序
{
  total_length_t_ = st_graph_data_.total_time_by_conf();//总时间
  unit_t_ = gridded_path_time_graph_config_.unit_t();//时间分辨率1.0
  total_length_s_ = st_graph_data_.path_length();//s的总长度
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();//1.0
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();//0.1
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();//101
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
}

Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {
  static constexpr double kBounadryEpsilon = 1e-2;

  for (const auto& boundary : st_graph_data_.st_boundaries()) {//遍历每个障碍物边界，检查是否有碰撞危险
    // KeepClear obstacles not considered in Dp St decision
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    //如果起点处于障碍物内则返回一系列速度点（s一直为0）
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||//如果起始点0.0处于障碍物边界内
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&//0.01
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(
                         total_length_t_ / static_cast<double>(unit_t_))) +
                     1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {//初始化网格
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));//[s,t,v=0,a=0,da=0]
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }//障碍物检查结束

  if (!InitCostTable().ok()) {//初始化cost_table
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!InitSpeedLimitLookUp().ok()) {//将每个s值的速度限制放入speed_limit_by_index_中
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!CalculateTotalCost().ok()) {//计算cost，obs+距离+上一点的总cost+速度+加速度+jerk
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!RetrieveSpeedProfile(speed_data).ok()) {//通过最后的最有节点回溯速度曲线（与A*类似）
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {//1e-6
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;//离散时间
//以下代码即首先根据congig设置的s的个数计算预设s的长度
//如果预设长度小于总长度，在预设的维度加上剩余的维度
//如果大于，减去；；即根据unit得到s的维度
  double sparse_length_s =//稀疏s长度
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ =//剩余的个数
      sparse_length_s > std::numeric_limits<double>::epsilon()//最小非零浮点数
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;//离散s
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;//如果没有剩余s，则s的维度等于长度/s分辨率
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));//初始化cost_table
//cost_table格式：是一个二次向量，第一维是t的个数，第二位是s的个数，对应的值为StGraphPoint
//[0  ]=[StGraphPoint][StGraphPoint]...[StGraphPoint]共N_s个
//[...]=...
//[N_t]=... 
  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {//对t进行遍历
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {//对config中的s维度进行cost_table初始化
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {//对剩余的s维度进行cost_table初始化
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = cost_table_[0];//第一个时刻的table
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);//t0时刻s的维度
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();//将每个s值放入spatial_distance_by_index_中
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();//得到速度限制

  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }//将每个s值的速度限制放入speed_limit_by_index_中
  return Status::OK();
}

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {//列
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {//初始为1
      std::vector<std::future<void>> results;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {//根据前一时刻计算下一时刻的s范围
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {//false
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
          CalculateCostAt(msg);
        }
      }
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {//false
        for (auto& result : results) {
          result.get();
        }
      }
    }

    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);//根据当前点的速度与s值以及单位时间计算下一时刻以最大加速度和最大减速度计算的最大和最小s值
        //即下一时刻的s值范围
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();//根据最大加速度计算单位时间后的s值
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }//通过当前点速度与时间计算下一个时间的最大s值

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();//同理，以最大减速度计算单位时间t后的s值
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }//最小s值
}

void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r];//StGraphPoint

  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));//计算该点处障碍物的cost（与障碍物之间的距离）
  //遍历所有障碍物，若计算cost=1.0*1e4*dx^2
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }

  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));//计算与目标的距离的cost
  //cost=(length-s)*1e2

  const auto& cost_init = cost_table_[0][0];//StGraphPoint
  if (c == 0) {//第一列，第一个时间t，起始点的cost为0，且限速为起始速度
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  if (c == 1) {//第二个时间t
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;
    if (acc < max_deceleration_ || acc > max_acceleration_) {//
      return;
    }

    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&//速度小于0
        cost_cr.point().s() > min_s_consider_speed) {//该点s大于1.75m
      return;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }//检测是否有障碍物存在于起始点与该点的连线上
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;
  }

  static constexpr double kSpeedRangeBuffer = 0.20;
  const double pre_lowest_s =
      cost_cr.point().s() -
      FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1;
  const auto& pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;

  if (c == 2) {//第二列
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
           pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;
  }

  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                              curr_a * unit_t_);
    }
  }
}

Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  for (const StGraphPoint& cur_point : cost_table_.back()) {//最后一个时刻的s
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }//找到最后一列中所有s的最小cost,将其作为best_end_point

  for (const auto& row : cost_table_) {//遍历所有的列
    const StGraphPoint& cur_point = row.back();//该列的最后一个点
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }//找到最后一行中最小的cost,并与最后一个时刻的最小cost做对比，更小的将其作为best_end_point

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {//与a*算法会回溯一样，通过最优的最后一个点，一步步选择它的父节点
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}

double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,//速度差值/时间
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +//加速度的平方*惩罚因子
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);//两个点人的jerk
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
