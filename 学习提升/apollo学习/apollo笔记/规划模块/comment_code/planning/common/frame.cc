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
 * @file frame.cc
 **/
#include "modules/planning/common/frame.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/feature_output.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/planning/proto/path_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::cyber::Clock;
using apollo::prediction::PredictionObstacles;

DrivingAction Frame::pad_msg_driving_action_ = DrivingAction::NONE;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : Frame(sequence_num, local_view, planning_start_point, vehicle_state,
            nullptr) {}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

bool Frame::Rerouting(PlanningContext *planning_context) {
  if (FLAGS_use_navigation_mode) {
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }
  if (local_view_.routing == nullptr) {
    AERROR << "No previous routing available";
    return false;
  }
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }
  auto request = local_view_.routing->routing_request();
  request.clear_header();

  auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
  double s = 0.0;
  double l = 0.0;
  hdmap::LaneInfoConstPtr lane;
  if (hdmap_->GetNearestLaneWithHeading(point, 5.0, vehicle_state_.heading(),
                                        M_PI / 3.0, &lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << point.DebugString() << ", heading:" << vehicle_state_.heading();
    return false;
  }
  request.clear_waypoint();
  auto *start_point = request.add_waypoint();
  start_point->set_id(lane->id().id());
  start_point->set_s(s);
  start_point->mutable_pose()->CopyFrom(point);
  for (const auto &waypoint : future_route_waypoints_) {
    // reference_line_provider_->FutureRouteWaypoints()) {
    request.add_waypoint()->CopyFrom(waypoint);
  }
  if (request.waypoint_size() <= 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }

  auto *rerouting =
      planning_context->mutable_planning_status()->mutable_rerouting();
  rerouting->set_need_rerouting(true);
  *rerouting->mutable_routing_request() = request;

  monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;
}

const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

// const std::list<ReferenceLineInfo> Frame::get_reference_line_info() const {
//   std::list<ReferenceLineInfo> dt_reference_line_info_(reference_line_info_.begin(),reference_line_info_.end());
//   return dt_reference_line_info_;
// }
std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}

bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {//遍历每条引导线,每段引导线分开存放
    if (segments_iter->StopForDestination()) {//在pnc_map中会判断下一个waypoint是否为目标点，是为true,检测终点是否在引导线中
      is_near_destination_ = true;
    }
    //将每条引导线对应的汽车位置（固定），开始规划点（固定）以及引导线，lansegment放入reference_line_info_
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }//遍历结束

  if (reference_line_info_.size() == 2) {//如果只有两条引导线
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
    common::SLPoint first_sl;
    //如果汽车在第一条引导线上没有投影值，则失败
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    //如果汽车在第二条引导线上没有投影值，则失败
    common::SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l() - second_sl.l();//求解两个投影值间的差值
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);//将差值赋给每条引导线对应的offset_to_other_reference_line_
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {//遍历所有的引导线info
  //初始化该引导线：检测与各种类型相交的区域，并将障碍物box(与轨迹box)投影值引导线，增加速度限制
  //生成障碍物的stboundary,sl_boundary,并检测是否会阻挡lane,is_lane_blocking_为true或false,
  //并根据障碍物是否超出引导线，与障碍物在车后，添加横纵向忽略决策
  //将障碍物的以上信息放入path_decision中
  //初始化ref_info信息，若是车与引导线超过10m,则会导致初始化失败
    if (!ref_info.Init(obstacles())) {
      AERROR << "Failed to init reference line";
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  static constexpr double kStopWallWidth = 4.0;
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

Status Frame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,//汽车状态
    const std::list<ReferenceLine> &reference_lines,//引导线
    const std::list<hdmap::RouteSegments> &segments,//生成引导线的routesegments
    const std::vector<routing::LaneWaypoint> &future_route_waypoints,//之后的waypoints向量
    const EgoInfo *ego_info) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.
  //首先构建障碍物信息(将障碍物的id和obstacle放入obstacles_={id，obstacle})，并检查障碍物是否会和汽车相撞
  //之后读取红绿灯信息，将其根据id放入traffic_lights中，
  //最后读取驾驶员意图，并赋值
  auto status = InitFrameData(vehicle_state_provider, ego_info);
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
//初始化引导线INFO(后面规划都会用到该信息，判断是否是终点引导线)：检测与各种类型相交的区域，
//并将障碍物投影值引导线，增加速度限制
  //初始化该引导线：检测与各种类型相交的区域，并将障碍物box(与轨迹box)投影值引导线，增加速度限制
  //生成障碍物的stboundary,sl_boundary,并检测是否会阻挡lane,is_lane_blocking_为true或false,
  //并根据障碍物是否超出引导线，与障碍物在车后，添加横纵向忽略决策
  //将障碍物的以上信息放入path_decision中
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  future_route_waypoints_ = future_route_waypoints;//之后的waypoints
  return Status::OK();
}

Status Frame::InitForOpenSpace(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  return InitFrameData(vehicle_state_provider, ego_info);
}

Status Frame::InitFrameData(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();//获得高清地图
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = vehicle_state_provider->vehicle_state();//获得车辆状态
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "Adc init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  if (FLAGS_align_prediction_time) {//false
    auto prediction = *(local_view_.prediction_obstacles);
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
    local_view_.prediction_obstacles->CopyFrom(prediction);
  }
  //构建障碍物
  //根据预测信息创建障碍物指针列表，并将其添加到frame中的obstacles_中
  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);//将障碍物的id和obstacle放入obstacles_={id，obstacle}
  }
  //检查汽车是否会和障碍物碰撞，通过box相交的方式
  //汽车速度为零时
  if (planning_start_point_.v() < 1e-3) {//planning_start_point_是开始规划的点
    const auto *collision_obstacle = FindCollisionObstacle(ego_info);//检测碰撞的障碍物，检测障碍物与ego信息box是否相交
    if (collision_obstacle != nullptr) {
      const std::string msg = absl::StrCat(
          "Found collision with obstacle: ", collision_obstacle->Id());
      AERROR << msg;
      monitor_logger_buffer_.ERROR(msg);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
//将红绿灯信息根据id放入traffic_lights_中，traffic_lights_[traffic_light.id]=traffic_light
  ReadTrafficLights();//读红绿灯

  ReadPadMsgDrivingAction();//将驾驶员行动赋值pad_msg_driving_action_

  return Status::OK();
}

const Obstacle *Frame::FindCollisionObstacle(const EgoInfo *ego_info) const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  const auto &adc_polygon = Polygon2d(ego_info->ego_box());
  for (const auto &obstacle : obstacles_.Items()) {//遍历所有障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_debug_data = debug->mutable_planning_data();
  auto *adc_position = planning_debug_data->mutable_adc_position();
  adc_position->CopyFrom(*local_view_.localization_estimate);

  auto debug_chassis = planning_debug_data->mutable_chassis();
  debug_chassis->CopyFrom(*local_view_.chassis);

  if (!FLAGS_use_navigation_mode) {
    auto debug_routing = planning_debug_data->mutable_routing();
    debug_routing->CopyFrom(*local_view_.routing);
  }

  planning_debug_data->mutable_prediction_header()->CopyFrom(
      local_view_.prediction_obstacles->header());
  /*
  auto relative_map = AdapterManager::GetRelativeMap();
  if (!relative_map->Empty()) {
    planning_debug_data->mutable_relative_map()->mutable_header()->CopyFrom(
        relative_map->GetLatestObserved().header());
  }
  */
}

void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles *prediction_obstacles) {
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

void Frame::ReadTrafficLights() {
  traffic_lights_.clear();

  const auto traffic_light_detection = local_view_.traffic_light;//接收到的红绿灯信息
  if (traffic_light_detection == nullptr) {
    return;
  }
  //交通灯检测到的时间到目前时间的差值，查看时间延时
  const double delay = traffic_light_detection->header().timestamp_sec() -
                       Clock::NowInSeconds();

  if (delay > FLAGS_signal_expire_time_sec) {//5.0
    ADEBUG << "traffic signals msg is expired, delay = " << delay
           << " seconds.";
    return;
  }
  for (const auto &traffic_light : traffic_light_detection->traffic_light()) {//
    traffic_lights_[traffic_light.id()] = &traffic_light;//将红绿灯信息根据id放入traffic_lights_中
  }
}

perception::TrafficLight Frame::GetSignal(
    const std::string &traffic_light_id) const {
  const auto *result =
      apollo::common::util::FindPtrOrNull(traffic_lights_, traffic_light_id);
  if (result == nullptr) {
    perception::TrafficLight traffic_light;
    traffic_light.set_id(traffic_light_id);
    traffic_light.set_color(perception::TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *result;
}

void Frame::ReadPadMsgDrivingAction() {//驾驶员意图
  if (local_view_.pad_msg) {
    if (local_view_.pad_msg->has_action()) {
      pad_msg_driving_action_ = local_view_.pad_msg->action();
    }
  }
}

void Frame::ResetPadMsgDrivingAction() {
  pad_msg_driving_action_ = DrivingAction::NONE;
}

// void Frame::get_pathboundary(planning_internal::Debug* debug_chart) 
// {
  // for (const auto reference_line_info : reference_line_info_) 
  // {
  //   for(auto& boundary : reference_line_info.GetCandidatePathBoundaries())
  //   //每条参考线有多个候选边界，遍历每个候选边界
  //   {
  //     //将每个候选边界的信息加入到path_boundary中
  //     Path_Boundary path_boundary_;
  //     path_boundary_=debug_chart->mutable_planning_data().add_path_boundary();
  //     path_boundary_.set_start_s(boundary.start_s());
  //     path_boundary_.set_delta_s(boundary.delta_s());
  //     //遍历该候选边界的boundary,将其放入
  //     for(auto edge : boundary.boundary())
  //     {
  //       Boundary vec_bound = path_boundary_->add_boundary();
  //       vec_bound.set_left_l(edge.first);
  //       vec_bound.set_right_l(edge.second);
  //     }
  //     path_boundary_.set_label(boundary.label());
  //   }
  // }
// }

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo *target_reference_line_info = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}

const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  for (const auto &reference_line_info : reference_line_info_) {
    // Find the unsuccessful lane-change path
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

}  // namespace planning
}  // namespace apollo
