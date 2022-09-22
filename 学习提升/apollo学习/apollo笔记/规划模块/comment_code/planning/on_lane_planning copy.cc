/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/on_lane_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "gtest/gtest_prod.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_semantic_map_config.pb.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/traffic_rules/traffic_decider.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/planning/proto/path_boundary.pb.h"


namespace apollo {
namespace planning {
using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::dreamview::Chart;
using apollo::hdmap::HDMapUtil;
using apollo::planning_internal::SLFrameDebug;
using apollo::planning_internal::SpeedPlan;
using apollo::planning_internal::STGraphDebug;
using apollo::routing::RoutingRequest;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::common::math::Polygon2d;
using apollo::common::PointENU;

OnLanePlanning::~OnLanePlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_routing_.Clear();
  injector_->ego_info()->Clear();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

Status OnLanePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) { //检查dag对应的配置文件
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }
//检查配置信息

  PlanningBase::Init(config_);//执行父类的初始化，
  //注册对应的规划状态的决策与优化器，对应具体的task的路径规划决策与优化，与论文对应

  planner_dispatcher_->Init();
  //planner_dispatcher_在on_lane_planning的构造函数中指向OnLanePlannerDispatcher
  //执行的是planner_dispatcher_的init函数，其父类的初始化函数
  //注册对应的规划器，即初始化四种类型的规划器（RTK，navi,lattice，public_road）,
  //将其放入producer中，在common::util::factory中

  ACHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;
      //从Proto文件获取交通规则，放入到traffic_rule_configs；
      //对应多个config，包含规则id以及对车辆的多种限制信息

  // clear planning history
  injector_->history()->Clear();//清楚规划历史，只执行一次

  // clear planning status
  injector_->planning_context()->mutable_planning_status()->Clear();

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";//hdmap_

  // instantiate reference line provider
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), hdmap_);
  //根据汽车状态以及高清地图实例化参考线规划器
 //将hdmap_赋值到pnc_map，并分配对应的smoother,指向为discrete_points_reference_line_smoother，并设置config
 //pnc_map就是hdmap与planning模块的中间层，用来处理routing结果，将routing的每个路由段转换为reference_lin的数据形式

  reference_line_provider_->Start();//开启参考线线程，根据routing结果生成reference_lines
  
  // dispatch ，分配具体的规划器
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  //根据配置为onlane_planning设置规划器，
  //默认为PUBLIC_ROAD,构造函数是将injector_赋值到它的父类中，PlannerWithReferenceLine
  //PlannerWithReferenceLine的构造函数将injector赋值它的父类Planner中，
  //planner构造函数，使用injector调用ScenarioManager的构造函数：将injector赋值
  //（有PUBLIC_ROAD（开放道路的轨迹规划器），RTK（根据录制的轨迹来规划行车路线）,LATTIC（基于网格算法的轨迹规划器）,NAVI（基于实时相对地图的规划器））
  
  if (!planner_) {//如果没有分配规划器，则失败
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    PlanningSemanticMapConfig renderer_config;
    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planning_birdview_img_feature_renderer_config_file,
        &renderer_config))
        << "Failed to load renderer config"
        << FLAGS_planning_birdview_img_feature_renderer_config_file;

    BirdviewImgFeatureRenderer::Instance()->Init(renderer_config);
  }

  start_time_ = Clock::NowInSeconds();//开始时间
  return planner_->Init(config_);
  //planner在126行中指向public_road_planner.cc
  //public_road_planner中无构造函数，
  //根据配置文件注册各种场景，并将所有场景的config放入config_map中
  //之后实例化lane_follow场景
}

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,//规划开始的点，缝合轨迹的最后一个点
                                 const VehicleState& vehicle_state) {
//调用frame构造函数，将形参数据信息信息传入,实例化frame类
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;

//获取引导线reference_lines，以及对应的routesegments
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    const std::string msg = "Failed to create reference line";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
//检查复制是否出现问题
  DCHECK_EQ(reference_lines.size(), segments.size());
//根据汽车速度计算前向距离，180-250m
  auto forward_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

//收缩引导线，根据设定的前行距离与后向距离，对其进行收缩，并将收缩的点放入reference_points_以及map_path中
//收缩时即先找到距离车在引导线上投影值s距离为FLAGS_look_backward_distance，forward_limit的引导线上点的index
//之后根据这两个index对引导线进行收缩
  for (auto& ref_line : reference_lines) {//同下
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  {//收缩每段seg的引导线
  for (auto& seg : segments) 
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                    FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink routing segments.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

//初始化坐标系，FutureRouteWaypoints()为从下一个waypoints到最后一个waypoints的vector
//首先构建障碍物信息，并检查障碍物是否会和汽车相撞
//之后读取红绿灯信息，将其根据id放入traffic_lights中
//最后读取驾驶员意图
//初始化引导线：检测与各种类型相交的区域，并将障碍物投影值引导线，增加速度限制
//将每条引导线对应的汽车位置（固定），开始规划点（固定）以及引导线，lansegment放入reference_line_info_
  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) {
  ptr_trajectory_pb->clear_trajectory_point();//清除轨迹点

  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();//获得目前车辆状态
  const double max_t = FLAGS_fallback_total_time;//最大时间，3.0
  const double unit_t = FLAGS_fallback_time_unit;//时间间隔，0.1

  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();//将车的位置设置为轨迹的第一个点
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {//以车的位置不断叠加relative_time,但位置不变
    tp.set_relative_time(t);
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

bool OnLanePlanning::JudgeCarInDeadEndJunction(
    std::vector<JunctionInfoConstPtr>* junctions,
    const Vec2d& car_position,
    JunctionInfoConstPtr* target_junction) {
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    return false;
  }
  for (size_t i = 0; i < junction_num; ++i) {
    if (junctions->at(i)->junction().type() == DEAD_END) {
      Polygon2d polygon = junctions->at(i)->polygon();
      // judge dead end point in the select junction
      if (polygon.IsPointIn(car_position)) {
        *target_junction = junctions->at(i);
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

bool OnLanePlanning::DeadEndHandle(
  const PointENU& dead_end_point,
  const VehicleState& vehicle_state) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();//获取高清地图
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(dead_end_point, 1.0, &junctions) != 0) {//从高清地图中返回距离dead_end_point_在1.0范围内的junction
    ADEBUG << "Fail to get junctions from base_map.";
    return false;
  }
  if (junctions.size() <= 0) {
    ADEBUG << "No junction from map";
    return false;
  }
  Vec2d car_position;
  car_position.set_x(vehicle_state.x());
  car_position.set_y(vehicle_state.y());
  //判断车是否在DEAD_END的junction中，是返回到junction中
  if (!JudgeCarInDeadEndJunction(&junctions, car_position, &junction)) {
    ADEBUG << "Target Dead End not found";
    return false;
  }
  return true;
}

void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
  // when rerouting, reference line might not be updated. In this case, planning
  // module maintains not-ready until be restarted.
  static bool failed_to_update_reference_line = false;
  local_view_ = local_view;  //local_view_保存的是底盘信息，定位信息，障碍物信息，交通灯信息等等
  const double start_timestamp = Clock::NowInSeconds();  //开始规划时间
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();//从系统时钟开始时计时
  // localization
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

  //根据定位信息以及底盘信息的时间戳关系，更新汽车状态
  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();

//得到引导线中的导航点数量
  size_t waypoint_num =
    local_view_.routing->routing_request().waypoint().size();

//设置dead_end_point_，导航点
  if (local_view_.routing->routing_request().dead_end_info().
    dead_end_routing_type() == routing::ROUTING_IN) {
//若导航类型为ROUTING_IN,则dead_end_point_就是waypoints的最后一个点
    dead_end_point_ = local_view_.routing->routing_request()
                    .waypoint().at(waypoint_num - 1).pose();
  } else if (local_view_.routing->routing_request().dead_end_info().
    dead_end_routing_type() == routing::ROUTING_OUT) {
//若导航类型为ROUTING_out,则dead_end_point_就是waypoints的第一个点
    dead_end_point_ = local_view_.routing->routing_request()
                    .waypoint().at(0).pose();
  }


//检查dead_point周围是否存在junction,且车附近是否存在在junction
//该语句不成立，因为wait_flag_==true
  if (DeadEndHandle(dead_end_point_, vehicle_state) && !wait_flag_) {
    // do not use reference line
    reference_line_provider_->Wait();
  }

  const double vehicle_state_timestamp = vehicle_state.timestamp();
//当开始规划时间小于汽车状态传来的时间时输出以下语句，目的是保证先更新汽车状态再开始规划
  DCHECK_GE(start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << start_timestamp - vehicle_state_timestamp << " secs";
//如果车辆状态更新失败，或者车辆状态数值有问题
//生成停车轨迹
  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);//什么意思
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);//设置head
//根据设定的时间，将车目前的位置设为终点，不断为该位置叠加dt，直到3s后，实现3s的强制停车
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }
  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {//0.02
//根据目前的时间以及车辆状态携带的时间戳，预测当前时刻汽车的位置，并将时间戳更新为当前时间
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

//如果两次导航信息不同，则重新初始化planner
  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    ADEBUG << "last_routing_:" << last_routing_.ShortDebugString();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);//更新routing信息
    planner_->Init(config_);//重新初始化规划器，注册场景，并默认public-road以及lane_follow
  }

//参考线线程停止或等待时，以及参考线生成失败时，为true
  failed_to_update_reference_line =
      (!reference_line_provider_->UpdatedReferenceLine());
  // early return when reference line fails to update after rerouting
//当参考线生成失败时，强制停止
  if (failed_to_update_reference_line) {
    const std::string msg = "Failed to update reference line after rerouting.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }
  // Update reference line provider and reset pull over if necessary
//为参考线生成器更新汽车状态
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);//根据规划频率(10hz,或者3hz)设定一次规划的时间

  std::string replan_reason;

//计算缝合轨迹,并且更新缝合轨迹的relative_time,若是状态有问题则根据汽车状态直接计算下一步的规划点起始点即可
  std::vector<TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          FLAGS_trajectory_stitching_preserved_length, true,//20
          last_publishable_trajectory_.get(), &replan_reason);

//设置规划起点，汽车状态以及起点box大小
  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
//序列++，运行一次加一次
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
//初始化Frame
//调用frame构造函数，将形参数据信息信息传入,实例化frame类
//收缩引导线，根据设定的前行距离与后向距离，对其进行收缩，并将收缩的点放入reference_points_以及map_path中
//构建障碍物信息，并检查障碍物是否会和汽车相撞
//读取红绿灯信息，将其根据id放入traffic_lights中
//读取驾驶员意图
//初始化引导线：检测与各种类型相交的区域，并将障碍物投影至引导线，增加速度限制
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  if (status.ok()) {//初始化成功，
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
  }//遍历所有障碍物寻找最近的障碍物

  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  //设置该轨迹初始化frame的时间
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {//初始化失败，强制停车
    AERROR << status.ToString();
    if (FLAGS_publish_estop) {//false
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      GenerateStopTrajectory(ptr_trajectory_pb);//产生停车轨迹（3s内在汽车本身位置停车）
    }
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);//GEAR_DRIVE？？？
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
    return;
  }
//挑出不符合交通规则的导航点
//reference_line_info_保存每条引导线对应的汽车位置（固定），开始规划点（固定）以及引导线，lansegment
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {//遍历所有引导线
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);//config赋值
    auto traffic_status =
        traffic_decider.Execute(frame_.get(), &ref_line_info, injector_);//根据交通灯的重叠区域与车位置的距离，设置不同的decelation
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);//设置该参考线的Driverable
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }
//开始规划
  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);//默认为PUBLIC_ROAD规划器

  for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
    ADEBUG << p.DebugString();
  }

  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();

  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();

    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    if (FLAGS_enable_planning_smoother) {//false
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);//优化
    }
  }

  // reference line recovery only one frame
  // bool complete_dead_end =
  //   frame_.get()->open_space_info().destination_reached();
  // AERROR << "complete_dead_end is: " << complete_dead_end;
  /*
  if (complete_dead_end) {
    reference_line_provider_->Start();
    wait_flag_ = true;
  }*/
  const uint32_t n = frame_->SequenceNum();
  injector_->frame_history()->Add(n, std::move(frame_));
}

void OnLanePlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    // store kappa and dkappa for performance evaluation
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;
    double dkappa_rms = 0.0;
    double kappa_max_abs = std::numeric_limits<double>::lowest();
    double dkappa_max_abs = std::numeric_limits<double>::lowest();
    for (const auto& reference_point : reference_points) {
      double kappa_sq = reference_point.kappa() * reference_point.kappa();
      double dkappa_sq = reference_point.dkappa() * reference_point.dkappa();
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = kappa_max_abs < kappa_sq ? kappa_sq : kappa_max_abs;
      dkappa_max_abs = dkappa_max_abs < dkappa_sq ? dkappa_sq : dkappa_max_abs;
    }
    double reference_points_size = static_cast<double>(reference_points.size());
    kappa_rms /= reference_points_size;
    dkappa_rms /= reference_points_size;
    kappa_rms = std::sqrt(kappa_rms);
    dkappa_rms = std::sqrt(dkappa_rms);
    rl_debug->set_kappa_rms(kappa_rms);
    rl_debug->set_dkappa_rms(dkappa_rms);
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();

    const double adc_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();
    const auto sample_s = 0.1;
    const auto reference_line_length =
        reference_line_info.reference_line().Length();
    double average_offset = 0.0;
    double sample_count = 0.0;
    for (double s = 0.0; s < reference_line_length; s += sample_s) {
      double left_width = reference_line_path.GetLaneLeftWidth(s);
      double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::abs(left_width - right_width);
      if (left_width < adc_half_width || right_width < adc_half_width) {
        is_off_road = true;
      }
      if (left_width < minimum_boundary) {
        minimum_boundary = left_width;
      }
      if (right_width < minimum_boundary) {
        minimum_boundary = right_width;
      }
      ++sample_count;
    }
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(average_offset / sample_count);
  }
}

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {

  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {//true
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());//初始规划点
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);//开始规划，默认为PUBLIC_ROAD

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

//pangminghui change to add path_boundary to ptr_trajectory_pb

auto* planning_data = ptr_debug->mutable_planning_data();
PathBoundary2* path_boundary;
for(const auto& reference_line_info : frame_.get()->reference_line_info())//遍历每条参考线
  {
    for(auto& boundary : reference_line_info.GetCandidatePathBoundaries())//每条参考线有多个候选边界，遍历每个候选边界
    {
      //将每个候选边界的信息加入到path_boundary中
      path_boundary = planning_data->add_path_boundary();
      path_boundary->set_start_s(boundary.start_s());
      path_boundary->set_delta_s(boundary.delta_s());
      //遍历该候选边界的boundary,将其放入
      Boundary* vec_bound;
      for(auto edge : boundary.boundary())
      {
        vec_bound = path_boundary->add_boundary();
        vec_bound->set_left_l(edge.second);//max_l
        vec_bound->set_right_l(edge.first);//min_l
      }
      path_boundary->set_label(boundary.label());
    }
  }
  AINFO << "===> planning component planning data: " << planning_data->DebugString();

  if (frame_->open_space_info().is_on_open_space_trajectory()) {//false
    frame_->mutable_open_space_info()->sync_debug_instance();
    const auto& publishable_trajectory =
        frame_->open_space_info().publishable_trajectory_data().first;
    const auto& publishable_trajectory_gear =
        frame_->open_space_info().publishable_trajectory_data().second;
    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
    ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

    // TODO(QiL): refine engage advice in open space trajectory optimizer.
    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

    // enable start auto from open_space planner.
    if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      engage_advice->set_reason(
          "Ready to engage when staring with OPEN_SPACE_PLANNER");
    } else {
      engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      engage_advice->set_reason("Keep engage while in parking");
    }
    // TODO(QiL): refine the export decision in open space info
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);

    if (FLAGS_enable_record_debug) {
      // ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
      ADEBUG << "Open space debug information added!";
      // call open space info load debug
      // TODO(Runxin): create a new flag to enable openspace chart
      ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                           ptr_debug);
    }
  } else {//进入
    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();//找到cost最小的参考线
    const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();//找到type改变的参考线
    if (!best_ref_info) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // Store current frame stitched path for possible speed fallback in next
    // frames
    DiscretizedPath current_frame_planned_path;
    for (const auto& trajectory_point : stitching_trajectory) {
      current_frame_planned_path.push_back(trajectory_point.path_point());//将缝合轨迹放入到current_frame_planned_path中
    }
    const auto& best_ref_path = best_ref_info->path_data().discretized_path();//获得点
    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),//将参考线的点拷贝到current_frame_planned_path中
              std::back_inserter(current_frame_planned_path));
    frame_->set_current_frame_planned_path(current_frame_planned_path);

    ptr_debug->MergeFrom(best_ref_info->debug());

    if (FLAGS_export_chart) {//false
      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
    } else {
      ExportReferenceLineDebug(ptr_debug);
      // Export additional ST-chart for failed lane-change speed planning
      const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
      if (failed_ref_info) {
        ExportFailedLaneChangeSTChart(failed_ref_info->debug(), ptr_debug);
      }
    }

    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());
    // set right of way status
    ptr_trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());

    for (const auto& id : best_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
    }

    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }

    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

    if (FLAGS_enable_rss_info) {
      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
    }

    best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                  injector_->planning_context());

    // Add debug information.
    if (FLAGS_enable_record_debug) {//true
      auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
      reference_line->set_name("planning_reference_line");

      const auto& reference_points =
          best_ref_info->reference_line().reference_points();
      double s = 0.0;
      double prev_x = 0.0;
      double prev_y = 0.0;
      bool empty_path = true;
      for (const auto& reference_point : reference_points) {
        auto* path_point = reference_line->add_path_point();
        path_point->set_x(reference_point.x());
        path_point->set_y(reference_point.y());
        path_point->set_theta(reference_point.heading());
        path_point->set_kappa(reference_point.kappa());
        path_point->set_dkappa(reference_point.dkappa());
        if (empty_path) {
          path_point->set_s(0.0);
          empty_path = false;
        } else {
          double dx = reference_point.x() - prev_x;
          double dy = reference_point.y() - prev_y;
          s += std::hypot(dx, dy);
          path_point->set_s(s);
        }
        prev_x = reference_point.x();
        prev_y = reference_point.y();
      }
    }

    last_publishable_trajectory_.reset(new PublishableTrajectory(
        current_time_stamp, best_ref_info->trajectory()));

    ADEBUG << "current_time_stamp: " << current_time_stamp;

    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));

    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

    best_ref_info->ExportEngageAdvice(
        ptr_trajectory_pb->mutable_engage_advice(),
        injector_->planning_context());
  }

  return status;
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  if (!config.standard_planning_config().has_planner_public_road_config()) {
    return false;
  }
  // TODO(All): check other config params
  return true;
}

void PopulateChartOptions(double x_min, double x_max, std::string x_label,
                          double y_min, double y_max, std::string y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddSTGraph(const STGraphDebug& st_graph, Chart* chart) {
  if (st_graph.name() == "DP_ST_SPEED_OPTIMIZER") {
    chart->set_title("Speed Heuristic");
  } else {
    chart->set_title("Planning S-T Graph");
  }
  PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 220.0, "s (meter)",
                       false, chart);

  for (const auto& boundary : st_graph.boundary()) {
    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("Speed Plan");
  PopulateChartOptions(0.0, 80.0, "s (meter)", 0.0, 50.0, "v (m/s)", false,
                       chart);

  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    // Set chartJS's dataset properties
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
}

void OnLanePlanning::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

void OnLanePlanning::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }
  AddSpeedPlan(src_data.speed_plan(), dst_data->add_chart());
}

void OnLanePlanning::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    AddPartitionedTrajectory(debug_info, debug_chart);
    AddStitchSpeedProfile(debug_chart);
    AddPublishedSpeed(trajectory_pb, debug_chart);
    AddPublishedAcceleration(trajectory_pb, debug_chart);
    // AddFallbackTrajectory(debug_info, debug_chart);
  }
}

void OnLanePlanning::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto open_space_debug = debug_info.planning_data().open_space();

  chart->set_title("Open Space Trajectory Optimizer Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
    obstacle_index += 1;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }

  auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("Smooth");
  size_t adc_label = 0;
  for (const auto& point : smoothed_trajectory.vehicle_motion_point()) {
    const auto x = point.trajectory_point().path_point().x();
    const auto y = point.trajectory_point().path_point().y();
    const auto heading = point.trajectory_point().path_point().theta();

    // Draw vehicle shape along the trajectory
    auto* adc_shape = chart->add_car();
    adc_shape->set_x(x);
    adc_shape->set_y(y);
    adc_shape->set_heading(heading);
    adc_shape->set_color("rgba(54, 162, 235, 1)");
    adc_shape->set_label(std::to_string(adc_label));
    adc_shape->set_hide_label_in_legend(true);
    ++adc_label;

    // Draw vehicle trajectory points
    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(x);
    point_debug->set_y(y);
  }

  // Set chartJS's dataset properties
  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";

  auto warm_start_trajectory = open_space_debug.warm_start_trajectory();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("WarmStart");
  for (const auto& point : warm_start_trajectory.vehicle_motion_point()) {
    auto* point_debug = warm_start_line->add_point();
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  // Set chartJS's dataset properties
  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

void OnLanePlanning::AddPartitionedTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_->vehicle_state();
  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned Trajectory");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  // Draw vehicle state
  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  // Draw the chosen trajectories
  const auto& chosen_trajectory = chosen_trajectories[0];
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_trajectory.trajectory_point()) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }
  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";

  // Draw partitioned trajectories
  size_t partitioned_trajectory_label = 0;
  for (const auto& partitioned_trajectory :
       open_space_debug.partitioned_trajectories().trajectory()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_trajectory_label));
    ++partitioned_trajectory_label;
    for (const auto& point : partitioned_trajectory.trajectory_point()) {
      auto* point_debug = partition_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";
  }

  // Draw trajectory stitching point (line with only one point)
  auto* stitching_line = chart->add_line();
  stitching_line->set_label("TrajectoryStitchingPoint");
  auto* trajectory_stitching_point = stitching_line->add_point();
  trajectory_stitching_point->set_x(
      open_space_debug.trajectory_stitching_point().path_point().x());
  trajectory_stitching_point->set_y(
      open_space_debug.trajectory_stitching_point().path_point().y());
  // Set chartJS's dataset properties
  auto* stitching_properties = stitching_line->mutable_properties();
  (*stitching_properties)["borderWidth"] = "3";
  (*stitching_properties)["pointRadius"] = "5";
  (*stitching_properties)["lineTension"] = "0";
  (*stitching_properties)["fill"] = "true";
  (*stitching_properties)["showLine"] = "true";

  // Draw fallback trajectory compared with the partitioned and potential
  // collision_point (line with only one point)
  if (open_space_debug.is_fallback_trajectory()) {
    auto* collision_line = chart->add_line();
    collision_line->set_label("FutureCollisionPoint");
    auto* future_collision_point = collision_line->add_point();
    future_collision_point->set_x(
        open_space_debug.future_collision_point().path_point().x());
    future_collision_point->set_y(
        open_space_debug.future_collision_point().path_point().y());
    // Set chartJS's dataset properties
    auto* collision_properties = collision_line->mutable_properties();
    (*collision_properties)["borderWidth"] = "3";
    (*collision_properties)["pointRadius"] = "8";
    (*collision_properties)["lineTension"] = "0";
    (*collision_properties)["fill"] = "true";
    (*stitching_properties)["showLine"] = "true";
    (*stitching_properties)["pointStyle"] = "cross";

    const auto& fallback_trajectories =
        open_space_debug.fallback_trajectory().trajectory();
    if (fallback_trajectories.empty() ||
        fallback_trajectories[0].trajectory_point().empty()) {
      return;
    }
    const auto& fallback_trajectory = fallback_trajectories[0];
    // has to define chart boundary first
    auto* fallback_line = chart->add_line();
    fallback_line->set_label("Fallback");
    for (const auto& point : fallback_trajectory.trajectory_point()) {
      auto* point_debug = fallback_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
    }
    // Set chartJS's dataset properties
    auto* fallback_properties = fallback_line->mutable_properties();
    (*fallback_properties)["borderWidth"] = "3";
    (*fallback_properties)["pointRadius"] = "2";
    (*fallback_properties)["lineTension"] = "0";
    (*fallback_properties)["fill"] = "false";
    (*fallback_properties)["showLine"] = "true";
  }
}

void OnLanePlanning::AddStitchSpeedProfile(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  const auto& last_trajectory =
      injector_->frame_history()->Latest()->current_frame_planned_trajectory();
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OnLanePlanning::AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                                       planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE) {
      point_debug->set_y(point.v());
    }
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE) {
      point_debug->set_y(-point.v());
    }
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

VehicleState OnLanePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                            const double curr_timestamp) const {
  // TODO(Jinyun): use the same method in trajectory stitching
  //               for forward prediction
  auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
      curr_timestamp - vehicle_state.timestamp());

  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

void OnLanePlanning::AddPublishedAcceleration(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("Acceleration (m/s^2)");

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());
  }
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

}  // namespace planning
}  // namespace apollo
