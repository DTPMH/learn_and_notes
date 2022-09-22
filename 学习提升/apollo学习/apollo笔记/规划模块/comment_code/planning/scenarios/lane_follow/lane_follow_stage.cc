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

/**
 * @file
 **/

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "modules/planning/tasks/deciders/path_decider/path_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

LaneFollowStage::LaneFollowStage(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {}

void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();

  const auto path_decision = reference_line_info->path_decision();
  for (const auto obstacle : path_decision->obstacles().Items()) {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

Stage::StageStatus LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();//输出参考线的个数

  unsigned int count = 0;

  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {//遍历每条参考线
    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line.";
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

//下面语句的作用是规划每条参考线，直到该参考线规划成功
    if (has_drivable_reference_line) {//上一条参考线规划成功，则下一条不规划(即一次只规划一条引导线)
      reference_line_info.SetDrivable(false);
      break;
    }
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);//主要的规划步骤
//得到轨迹，将其放入reference_line_info->trajectory

    if (cur_status.ok()) {//规划成功
      if (reference_line_info.IsChangeLanePath()) {//变道的引导线
        ADEBUG << "reference line is lane change ref.";
        ADEBUG << "FLAGS_enable_smarter_lane_change: "
               << FLAGS_enable_smarter_lane_change;
        if (reference_line_info.Cost() < kStraightForwardLineCost &&//10.0
            (LaneChangeDecider::IsClearToChangeLane(&reference_line_info) ||//检测是否可以变道
             FLAGS_enable_smarter_lane_change)) {//false
          // If the path and speed optimization succeed on target lane while
          // under smart lane-change or IsClearToChangeLane under older version
          has_drivable_reference_line = true;
          reference_line_info.SetDrivable(true);
          LaneChangeDecider::UpdatePreparationDistance(
              true, frame, &reference_line_info, injector_->planning_context());
          ADEBUG << "\tclear for lane change";
        } else {
          LaneChangeDecider::UpdatePreparationDistance(
              false, frame, &reference_line_info,
              injector_->planning_context());
          reference_line_info.SetDrivable(false);
          ADEBUG << "\tlane change failed";
        }
      } else {//车道直行，则设置该引导线可执行
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
      }
    } else {//规划失败
      reference_line_info.SetDrivable(false);//设置该参考线不能规划
    }
  }//引导线遍历结束，轨迹生成结束

  return has_drivable_reference_line ? StageStatus::RUNNING
                                     : StageStatus::ERROR;
}

Status LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {//车在当前引导线上，则加直行cost(目的优先变道)
    reference_line_info->AddCost(kStraightForwardLineCost);//cost+=10
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  ADEBUG << "Current reference_line_info is IsChangeLanePath: "
         << reference_line_info->IsChangeLanePath();

  auto ret = Status::OK();
  for (auto* task : task_list_) {//在构造函数中被构造，stage.cc
    const double start_timestamp = Clock::NowInSeconds();

    ret = task->Execute(frame, reference_line_info);
//首先根据lane_change_decider，检测是否需要变道，改变变道状态:in_change_lane,change_lane_succ,change_lane_fail
//改变injector中的change_lane的状态

//之后PathReuseDecider，（值为false,直接跳过）,该参考线的path_reusable为false

//之后是PATH_LANE_BORROW_DECIDER，判断是否需要借道（场景是前面的路被障碍物阻挡，需要从左边或者右边借道）
//设置参考线的set_is_path_lane_borrow为true，且改变mutable_path_decider_status = injector_->planning_context()
                                          //->mutable_planning_status()
                                          //->mutable_path_decider()
//mutable_path_decider_status->add_decided_side_pass_direction(PathDeciderStatus::RIGHT_BORROW)
//或者PathDeciderStatus::LEFT_BORROW

//之后是PATH_BOUNDS_DECIDER，生成路线的bound信息
//计算应急车道（fallback），靠边停车(pull-over)，change_lane(regular/change_lane)
//正常行驶(regular/self)，borrow(regular/left/right)的车道边界信息，
//并将其放入引导线的SetCandidatePathBoundaries中

//PIECEWISE_JERK_PATH_OPTIMIZER，根据上一步生成的bound对引导线中的点进行优化得到path点，(根据论文中内容进行优化)
//并将其放入引导线的SetCandidatePathData中

//之后PATH_ASSESSMENT_DECIDER，
//将每个候选path的点对其打上decision标签，并根据每条路径的长度选择最短的路径放入引导线的path_data中
//并重新将合法路径放入引导线的候选路径中CandidatePathData

//PATH_DECIDER,遍历参考线上的静态障碍物，做出自身决策，ignore，stop,nudge（left,right）

//RULE_BASED_STOP_DECIDER强制停车决策，根据规则设定停车标志

//ST_BOUNDS_DECIDER，初始化ST坐标系，并将障碍物映射到ST上，提取相应的STbound(细节)
//并将障碍物的boundary类型进行判定

//SPEED_BOUNDS_PRIORI_DECIDER（对应speed_bound_decider），精细确定STbound，并对每个点限速（细节）
//
//SPEED_HEURISTIC_OPTIMIZER（对应path_time_heuristic_optimizer），初始化ST图并计算t,s的cost，
//并根据最小的cost返回一条速度轨迹（与A*结果类似，通过回溯得到path）

//SPEED_DECIDER，根据每个障碍物的st_boundary判断其与车的位置，根据此设定对该障碍物的决策

//SPEED_BOUNDS_FINAL_DECIDER.将规划路径上障碍物得stboundary映射到路径对应的ST，并根据此设定速度的限制

//PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER，
//对上一步搜索出的速度轨迹进行平滑，从而得到一条平滑、舒适的速度轨迹
//RSS_DECIDER  //责任敏感安全模型：Responsibility Sensitive Safety
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "after task[" << task->Name()
           << "]:" << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }

    // TODO(SHU): disable reference line order changes for now
    // updated reference_line_info, because it is changed in
    // lane_change_decider by PrioritizeChangeLane().
    // reference_line_info = &frame->mutable_reference_line_info()->front();
    // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
    //        << reference_line_info->IsChangeLanePath();
  }//tasks执行结束

  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  if (!ret.ok()) {//如果某个task失败
    PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
  }
//生成fallback,path_data以及speed_data
  DiscretizedTrajectory trajectory;
  //首先根据时间，利用速度轨迹计算当前时刻的s值，之后根据s值利用路径轨迹计算当前点的x,y坐标，
  //将x,y，heading,曲率，速度，加速度等信息放入trajectory_points中
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);//将path与speed结合，形成新的trajectory
  }

  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {//遍历障碍物寻找是否有终点
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }//终点寻找结束

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {//再遍历障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {//查看静态障碍物的stop决策
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {//给没有终点的引导线以及存在终点但是终点前存在静态障碍物的引导线添加1000的cost
        static constexpr double kReferenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }

  if (FLAGS_enable_trajectory_check) {//检查轨迹
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  reference_line_info->SetTrajectory(trajectory);//设置轨迹
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

void LaneFollowStage::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // path and speed fall back
  if (reference_line_info->path_data().Empty()) {//如果没有path
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }//根据初始点的heading以及参考线分别生成离散的路劲点

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {//使用之前的frame的路径作为fallbcak路径
    if (!RetrieveLastFramePathProfile(
            reference_line_info, frame,
            reference_line_info->mutable_path_data())) {
      const auto& candidate_path_data =
          reference_line_info->GetCandidatePathData();
      for (const auto& path_data : candidate_path_data) {
        if (path_data.path_label().find("self") != std::string::npos) {
          *reference_line_info->mutable_path_data() = path_data;
          AERROR << "Use current frame self lane path as fallback ";
          break;
        }
      }
    }
  }

  AERROR << "Speed fallback due to algorithm failure";
  *reference_line_info->mutable_speed_data() =
      SpeedProfileGenerator::GenerateFallbackSpeed(
          injector_->ego_info(), reference_line_info->GetFallbackAcc());//产生速度轨迹

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }
}

void LaneFollowStage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {//如果没有投影值，则通过起始点的heading+单位s,计算离散轨迹
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      path_points.push_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const double max_s = reference_line.Length();
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));//根据参考线生成离散路径点
}

bool LaneFollowStage::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* path_data) {
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr) {
    AERROR
        << "Last frame doesn't succeed, fail to retrieve last frame path data";
    return false;
  }

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();

  path_data->SetDiscretizedPath(last_frame_discretized_path);
  const auto adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return false;
  }
  AERROR << "Use last frame good path to do speed fallback";
  return true;
}

SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
