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
 * @file
 **/

#include "modules/planning/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {//如果当前车的速度与加速度小于设定值
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
    //则通过当前速度与加速度和一次规划的时间计算下一个点
  } else {//若是大于设定值
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);//则先预测车辆状态
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);//之后再根据预测的车辆状态计算下一个规划点
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);//返回规划点
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
//汽车状态，开始规划的时间，一次规划的时间，上次缝合点的数量（20），是否补偿replan（true）,上一步轨迹，replan原因
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    std::string* replan_reason) {
      //FLAGS_enable_trajectory_stitcher==true
  if (!FLAGS_enable_trajectory_stitcher) {//若是禁止缝合轨迹，则通过车辆状态预测计算一个规划点
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);//则通过车辆状态预测计算一个规划点
  }
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";//如果replan的原因是上一步没有轨迹生成，则通过车辆状态预测计算下一个规划点
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {//如果汽车的驾驶室非自动驾驶模式，则预测计算下一个规划点
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();///上一步轨迹点的数量

  if (prev_trajectory_size == 0) {//如果没有上一步轨迹，则同上，预测计算下一个规划点
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();//

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);//从时间上找到上一步轨迹距离当前时间最近的点

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (time_matched_index + 1 >= prev_trajectory_size) {//如果时间上最近的点是上一步轨迹的最后一个点，
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);//则重新计算规划点
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));//得到该点的坐标等信息

  if (!time_matched_point.has_path_point()) {//该点格式错误
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);//查询距离汽车最近的点的index

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));//计算车投影到该路径点的s,l值

  if (replan_by_offset) {//true
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;//S的距离
    AERROR<<"time_matched_point :s : "<<time_matched_point.path_point().s();
    AERROR<<"project :s : "<<frenet_sd.first;
    auto lat_diff = frenet_sd.second;//L距离

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {//如果L距离大于设定值
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);//同样通过预测计算规划点
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {//S的值大于设定值
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,//通过预测计算规划点
                                              vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }
//
  double forward_rel_time = veh_rel_time + planning_cycle_time;//计算上一步规划到现在规划的差值时间+每次规划的时间，可以计算出这一次规划的总时间

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);//计算距离这一次规划最近的上一步轨迹点的序号
    //  *--------*########################*
    //           ########################*-----------。
    //  
  auto matched_index = std::min(time_matched_index, position_matched_index);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();//缝合轨迹中的点的数量

  const double zero_s = stitching_trajectory.back().path_point().s();//缝合轨迹中最后一个点的S值
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
  //  new_relative_time = old_realtive-dt = 
  //                                      old_realtive-(current_timestamp-prev_trajectory->header_time())
  //                                      =relative_time() + prev_trajectory->header_time() - current_timestamp
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);//设定S值
  }
  return stitching_trajectory;//返回缝合轨迹
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();//s值，InnerProd计算内积
  frenet_sd.second = v.CrossProd(n);//l
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
