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

#include "modules/planning/scenarios/scenario_manager.h"

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_pull_over/emergency_pull_over_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/scenarios/yield_sign/yield_sign_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);
  RegisterScenarios();//注册场景，并将其对应的config放入config_map中
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;//默认类型为车道线保持
  current_scenario_ = CreateScenario(default_scenario_type_);//根据默认类型创建当前的场景
  //构造相应的场景，首先执行父类的构造函数，Scenario：将config等赋值并得到场景名字
  //子类LaneFollowScenario构造函数为空
  //根据场景初始化，调用Scenario::Init()创建对应场景的stage
  //之后在Scenario::Init()函数中对stage进行实例化，调用stage.cc构造函数对task进行初始化
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
      ptr.reset(
          new scenario::bare_intersection::BareIntersectionUnprotectedScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::EMERGENCY_PULL_OVER:
      ptr.reset(new emergency_pull_over::EmergencyPullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::EMERGENCY_STOP:
      ptr.reset(new emergency_stop::EmergencyStopScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(
          config_map_[scenario_type], &scenario_context_, injector_));//默认为lane_follow,调用LaneFollowScenario的构造函数
      break;
    case ScenarioConfig::LEARNING_MODEL_SAMPLE:
      ptr.reset(new scenario::LearningModelSampleScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::PARK_AND_GO:
      ptr.reset(new scenario::park_and_go::ParkAndGoScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::PULL_OVER:
      ptr.reset(new scenario::pull_over::PullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      ptr.reset(new scenario::stop_sign::StopSignUnprotectedScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
      ptr.reset(new scenario::traffic_light::TrafficLightProtectedScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedLeftTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedRightTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::VALET_PARKING:
      ptr.reset(new scenario::valet_parking::ValetParkingScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::YIELD_SIGN:
      ptr.reset(new scenario::yield_sign::YieldSignScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::DEADEND_TURNAROUND:
      ptr.reset(new scenario::deadend_turnaround::DeadEndTurnAroundScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  if (planning_config_.learning_mode() == PlanningConfig::HYBRID ||
      planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST) {
    // HYBRID or HYBRID_TEST
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_hybrid_config_file,
                                &config_map_[ScenarioConfig::LANE_FOLLOW]));

  } else {
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                                &config_map_[ScenarioConfig::LANE_FOLLOW]));
                                //将lane_follow对应的config放到config_map_对应的hash表中
                                //配置文件处于lane_follow_hybrid_config.pb.txt
  }

//将各种场景的config加载
  // bare_intersection
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_bare_intersection_unprotected_config_file,
      &config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]));

  // emergency_pull_over
  ACHECK(
      Scenario::LoadConfig(FLAGS_scenario_emergency_pull_over_config_file,
                           &config_map_[ScenarioConfig::EMERGENCY_PULL_OVER]));

  // emergency_stop
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_emergency_stop_config_file,
                              &config_map_[ScenarioConfig::EMERGENCY_STOP]));

  // learning model
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_learning_model_sample_config_file,
      &config_map_[ScenarioConfig::LEARNING_MODEL_SAMPLE]));

  // park_and_go
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_park_and_go_config_file,
                              &config_map_[ScenarioConfig::PARK_AND_GO]));

  // pull_over
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
                              &config_map_[ScenarioConfig::PULL_OVER]));

  // stop_sign
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_stop_sign_unprotected_config_file,
      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));

  // traffic_light
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_protected_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_left_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));

  // valet parking
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                              &config_map_[ScenarioConfig::VALET_PARKING]));

  // yield_sign
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_yield_sign_config_file,
                              &config_map_[ScenarioConfig::YIELD_SIGN]));
  // turn around
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_deadend_turnaround_config_file,
      &config_map_[ScenarioConfig::DEADEND_TURNAROUND]));
}

ScenarioConfig::ScenarioType ScenarioManager::SelectPullOverScenario(
    const Frame& frame) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::PULL_OVER].pull_over_config();

  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;//车头距离目标点的距离
  ADEBUG << "adc_distance_to_dest[" << adc_distance_to_dest
         << "] destination_s[" << dest_sl.s() << "] adc_front_edge_s["
         << adc_front_edge_s << "]";

//当只有一条引导线且车头距离目标的距离大于10m小于50m时触发pull_over场景
  bool pull_over_scenario =
      (frame.reference_line_info().size() == 1 &&  // NO, while changing lane
       adc_distance_to_dest >=
           scenario_config.pull_over_min_distance_buffer() &&//10.0
       adc_distance_to_dest <=
           scenario_config.start_pull_over_scenario_distance());//50

  // too close to destination + not found pull-over position
  //当距离目标点距离小于25m时仍没有找点停车点坐标，则触发pull_over失败
  if (pull_over_scenario) {
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    if (adc_distance_to_dest < scenario_config.max_distance_stop_search() &&//25，距离终点小于25m且仍没有靠边停车的目标点，返回false
        !pull_over_status.has_position()) {
      pull_over_scenario = false;
    }
  }

  // check around junction
  if (pull_over_scenario) {
    static constexpr double kDistanceToAvoidJunction = 8.0;  // meter
    for (const auto& overlap : first_encountered_overlap_map_) {
      if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
          overlap.first == ReferenceLineInfo::SIGNAL ||
          overlap.first == ReferenceLineInfo::STOP_SIGN ||
          overlap.first == ReferenceLineInfo::YIELD_SIGN) {
        const double distance_to = overlap.second.start_s - dest_sl.s();
        const double distance_passed = dest_sl.s() - overlap.second.end_s;
        if ((distance_to > 0.0 && distance_to < kDistanceToAvoidJunction) ||//终点前8米内有以上区域时停止
            (distance_passed > 0.0 &&
             distance_passed < kDistanceToAvoidJunction)) {//终点后8内有以上区域，均不触发靠边停车
          pull_over_scenario = false;
          break;
        }
      }
    }
  }

  // check rightmost driving lane along pull-over path
  //从车当前位置到终点前每隔5m检测一次最右边车道能否停车，若存在不能停车区域，则pull_over触发失败
  if (pull_over_scenario) {
    double check_s = adc_front_edge_s;
    static constexpr double kDistanceUnit = 5.0;
    while (check_s < dest_sl.s()) {//yi 5m为单位向前查询
      check_s += kDistanceUnit;//分辨率为5m

      std::vector<hdmap::LaneInfoConstPtr> lanes;
      reference_line.GetLaneFromS(check_s, &lanes);
      if (lanes.empty()) {
        ADEBUG << "check_s[" << check_s << "] can't find a lane";
        continue;
      }
      const hdmap::LaneInfoConstPtr lane = lanes[0];
      const std::string lane_id = lane->lane().id().id();
      ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

      // check neighbor lanes type: NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
      bool rightmost_driving_lane = true;
      for (const auto& neighbor_lane_id :
           lane->lane().right_neighbor_forward_lane_id()) {//右边停车
        const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
        CHECK_NOTNULL(hdmap_ptr);
        const auto neighbor_lane = hdmap_ptr->GetLaneById(neighbor_lane_id);
        if (neighbor_lane == nullptr) {
          ADEBUG << "Failed to find neighbor lane[" << neighbor_lane_id.id()
                 << "]";
          continue;
        }
        const auto& lane_type = neighbor_lane->lane().type();
        if (lane_type == hdmap::Lane::CITY_DRIVING) {
          ADEBUG << "lane[" << lane_id << "]'s right neighbor forward lane["
                 << neighbor_lane_id.id() << "] type["
                 << Lane_LaneType_Name(lane_type) << "] can't pull over";
          rightmost_driving_lane = false;
          break;
        }
      }
      if (!rightmost_driving_lane) {
        pull_over_scenario = false;
        break;
      }
    }//while结束
  }

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW://只能从lane_follow场景触发pull_over
      if (pull_over_scenario) {
        return ScenarioConfig::PULL_OVER;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::VALET_PARKING:
    case ScenarioConfig::DEADEND_TURNAROUND:
    case ScenarioConfig::YIELD_SIGN:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectPadMsgScenario(
    const Frame& frame) {
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();//根据local_view赋值
  switch (pad_msg_driving_action) {
    case DrivingAction::PULL_OVER:
      if (FLAGS_enable_scenario_emergency_pull_over) {//true
        return ScenarioConfig::EMERGENCY_PULL_OVER;
      }
      break;
    case DrivingAction::STOP:
      if (FLAGS_enable_scenario_emergency_stop) {//true
        return ScenarioConfig::EMERGENCY_STOP;
      }
      break;
    case DrivingAction::RESUME_CRUISE://如果驾驶员操作为恢复巡航，则若是当前场景为紧急停车或者是紧急靠边停车，则将场景配置为PARK and GO
      if (current_scenario_->scenario_type() ==
              ScenarioConfig::EMERGENCY_PULL_OVER ||
          current_scenario_->scenario_type() ==
              ScenarioConfig::EMERGENCY_STOP) {
        return ScenarioConfig::PARK_AND_GO;
      }
      break;
    default:
      break;
  }
  return default_scenario_type_;//以上条件都不满足，则返回默认场景
}

ScenarioConfig::ScenarioType ScenarioManager::SelectInterceptionScenario(
    const Frame& frame) {
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;

  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  hdmap::PathOverlap* pnc_junction_overlap = nullptr;
  ReferenceLineInfo::OverlapType overlap_type;

  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();//得到该引导线上各种交通区的重叠区域
  // note: first_encountered_overlaps already sorted
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL ||//交通灯
        overlap.first == ReferenceLineInfo::STOP_SIGN ||//停车标注
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {//减速
      overlap_type = overlap.first;
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {//路口
      pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
    }
  }

  // pick a closer one between consecutive bare_intersection and traffic_sign
  //在以上几种情况中选择一种更近的处理
  if (traffic_sign_overlap && pnc_junction_overlap) {
    static constexpr double kJunctionDelta = 10.0;
    double s_diff = std::fabs(traffic_sign_overlap->start_s -
                              pnc_junction_overlap->start_s);
    if (s_diff >= kJunctionDelta) {
      if (pnc_junction_overlap->start_s > traffic_sign_overlap->start_s) {
        pnc_junction_overlap = nullptr;
      } else {
        traffic_sign_overlap = nullptr;
      }
    }
  }

  if (traffic_sign_overlap) {//交通灯在前（优先处理交通灯）
    switch (overlap_type) {
      case ReferenceLineInfo::STOP_SIGN:
        if (FLAGS_enable_scenario_stop_sign) {
          scenario_type = SelectStopSignScenario(frame, *traffic_sign_overlap);//若是车头距离停车标注大于0小于4m时，返回
        }
        break;
      case ReferenceLineInfo::SIGNAL:
        if (FLAGS_enable_scenario_traffic_light) {
          scenario_type =
              SelectTrafficLightScenario(frame, *traffic_sign_overlap);
        }
        break;
      case ReferenceLineInfo::YIELD_SIGN:
        if (FLAGS_enable_scenario_yield_sign) {
          scenario_type = SelectYieldSignScenario(frame, *traffic_sign_overlap);
        }
        break;
      default:
        break;
    }
  } else if (pnc_junction_overlap) {//路口在前
    // bare intersection
    if (FLAGS_enable_scenario_bare_intersection) {
      scenario_type =
          SelectBareIntersectionScenario(frame, *pnc_junction_overlap);
    }
  }

  return scenario_type;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectStopSignScenario(
    const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]
          .stop_sign_unprotected_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap.start_s - adc_front_edge_s;//车头到停车标志的距离
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap.object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap.start_s << "]";

//当车头距离停车标注小于4m时，将其设为true
  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <=
           scenario_config.start_stop_sign_scenario_distance());//4.0
  const bool stop_sign_all_way = false;  // TODO(all)

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (stop_sign_scenario) {
        return stop_sign_all_way ? ScenarioConfig::STOP_SIGN_PROTECTED
                                 : ScenarioConfig::STOP_SIGN_UNPROTECTED;//返回
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectTrafficLightScenario(
    const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap) {
  // some scenario may need start sooner than the others
  const double start_check_distance = std::max(//得到处理交通灯的最大距离 ，5m,30m,5m
      {config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
           .traffic_light_protected_config()
           .start_traffic_light_scenario_distance(),
       config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]
           .traffic_light_unprotected_left_turn_config()
           .start_traffic_light_scenario_distance(),
       config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]
           .traffic_light_unprotected_right_turn_config()
           .start_traffic_light_scenario_distance()});

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // find all the traffic light belong to
  // the same group as first encountered traffic light
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  for (const auto& overlap : traffic_light_overlaps) {
    const double dist = overlap.start_s - traffic_light_overlap.start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      next_traffic_lights.push_back(overlap);
    }
  }

  bool traffic_light_scenario = false;
  bool red_light = false;
  bool left_turn_signal = false;
  const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& traffic_light_overlap : next_traffic_lights) {
    const double adc_distance_to_traffic_light =//traffic_light到车头的距离
        traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s
           << "] adc_distance_to_traffic_light["
           << adc_distance_to_traffic_light << "]";

    // enter traffic-light scenarios: based on distance only
    //只考虑处于0-30m的交通灯区域
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {//30
      continue;
    }

    traffic_light_scenario = true;

    const auto& signal_color =
        frame.GetSignal(traffic_light_overlap.object_id).color();
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "]";
    apollo::hdmap::Id signal_id;
    signal_id.set_id(traffic_light_overlap.object_id);
    auto signal = hdmap_ptr->GetSignalById(signal_id)->signal();
    for (auto subsignal : signal.subsignal()) {
      if (subsignal.type() == apollo::hdmap::Subsignal::ARROW_LEFT) {
        left_turn_signal = true;
        break;
      }
    }
    if (signal_color != perception::TrafficLight::GREEN) {
      red_light = true;
      break;
    }
  }//交通灯区域遍历结束

  bool traffic_light_protected_scenario = false;
  bool traffic_light_unprotected_left_turn_scenario = false;
  bool traffic_light_unprotected_right_turn_scenario = false;
  if (traffic_light_scenario) {
    const auto& turn_type =
        reference_line_info.GetPathTurnType(traffic_light_overlap.start_s);//检查交通灯开始点的lane的type
    const bool right_turn = (turn_type == hdmap::Lane::RIGHT_TURN);
    const bool left_turn = (turn_type == hdmap::Lane::LEFT_TURN);
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;

    if (right_turn && red_light) {//如果该点处lane的type是右转且为红灯，则
      // check TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN
      const auto& scenario_config =
          config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]
              .traffic_light_unprotected_right_turn_config();
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) {//5m,且车与其距离小于5m则选择场景为traffic_light_unprotected_right_turn_scenario
        traffic_light_unprotected_right_turn_scenario = true;
      }
    } else if (left_turn && !left_turn_signal) {//该点处lane的type为左转，但subsignal不是left_Arrow
      // check TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN
      const auto& scenario_config =
          config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]
              .traffic_light_unprotected_left_turn_config();
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) {//30，且距离在30m内
        traffic_light_unprotected_left_turn_scenario = true;
      }
    } else {
      // check TRAFFIC_LIGHT_PROTECTED
      const auto& scenario_config =
          config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
              .traffic_light_protected_config();
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) {
        traffic_light_protected_scenario = true;
      }
    }
  }//if结束

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (traffic_light_unprotected_left_turn_scenario) {
        return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN;
      } else if (traffic_light_unprotected_right_turn_scenario) {
        return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
      } else if (traffic_light_protected_scenario) {
        return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;

    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectYieldSignScenario(
    const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::YIELD_SIGN].yield_sign_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_yield_sign =
      yield_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_yield_sign[" << adc_distance_to_yield_sign
         << "] yield_sign[" << yield_sign_overlap.object_id
         << "] yield_sign_overlap_start_s[" << yield_sign_overlap.start_s
         << "]";

  const bool yield_sign_scenario =
      (adc_distance_to_yield_sign > 0.0 &&
       adc_distance_to_yield_sign <=
           scenario_config.start_yield_sign_scenario_distance());//10m

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (yield_sign_scenario) {
        return ScenarioConfig::YIELD_SIGN;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectBareIntersectionScenario(
    const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  if (reference_line_info.GetIntersectionRightofWayStatus(
          pnc_junction_overlap)) {
    return default_scenario_type_;
  }

  const auto& scenario_config =
      config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]
          .bare_intersection_unprotected_config();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap.object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap.start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0.0 &&
       adc_distance_to_pnc_junction <=
           scenario_config.start_bare_intersection_scenario_distance());//25m

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (bare_junction_scenario) {
        return ScenarioConfig::BARE_INTERSECTION_UNPROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectValetParkingScenario(
    const Frame& frame) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::VALET_PARKING].valet_parking_config();

  // TODO(All) trigger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      scenario_config.parking_spot_range_to_start();//20
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioConfig::VALET_PARKING;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectDeadEndScenario(
    const Frame& frame) {
  size_t waypoint_num =
    frame.local_view().routing->routing_request().waypoint().size();
  const auto& routing_type =
    frame.local_view().routing->routing_request().dead_end_info().
    dead_end_routing_type();
  if (routing_type == routing::ROUTING_IN) {
    dead_end_point_ = frame.local_view().routing->routing_request().
                    waypoint().at(waypoint_num - 1).pose();
  } else if (routing_type == routing::ROUTING_OUT) {
    dead_end_point_ = frame.local_view().routing->routing_request().
                    waypoint().at(0).pose();
  }
  const auto& scenario_config =
    config_map_[ScenarioConfig::DEADEND_TURNAROUND].deadend_turnaround_config();
  double dead_end_start_range =
      scenario_config.dead_end_start_range();
  if (scenario::deadend_turnaround::DeadEndTurnAroundScenario::IsTransferable(
          frame, dead_end_point_, dead_end_start_range) &&
          routing_type == routing::ROUTING_IN) {
    return ScenarioConfig::DEADEND_TURNAROUND;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectParkAndGoScenario(
    const Frame& frame) {
  bool park_and_go = false;
  const auto& scenario_config =
      config_map_[ScenarioConfig::PARK_AND_GO].park_and_go_config();
  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  // TODO(SHU) might consider gear == GEAR_PARKING
  double adc_speed = vehicle_state_provider->linear_velocity();
  double s = 0.0;
  double l = 0.0;
  const double max_abs_speed_when_stopped =
      common::VehicleConfigHelper::Instance()
          ->GetConfig()
          .vehicle_param()
          .max_abs_speed_when_stopped();//0.2

  hdmap::LaneInfoConstPtr lane;

  // check ego vehicle distance to destination
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());//最后一个航路点
  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();//第一条引导线
  reference_line.XYToSL(routing_end.pose(), &dest_sl);//航路点在第一条引导线的sl值
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();//车头在引导线上的s值

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;//目标点与汽车的距离
  // if vehicle is static, far enough to destination and (off-lane or not on
  // city_driving lane)
  if (std::fabs(adc_speed) < max_abs_speed_when_stopped &&
      adc_distance_to_dest > scenario_config.min_dist_to_dest() &&//25
      (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
           adc_point, 2.0, vehicle_state.heading(), M_PI / 3.0, &lane, &s,
           &l) != 0 ||
       lane->lane().type() != hdmap::Lane::CITY_DRIVING)) {
    park_and_go = true;//如果当前速度小于0.2，且目标与汽车之间的距离大于25m，
    //且车在2.0米内有邻近车道或者lane时非驾驶车道时，则将其设置为park_and_go场景
  }

  if (park_and_go) {
    return ScenarioConfig::PARK_AND_GO;
  }

  return default_scenario_type_;
}

void ScenarioManager::Observe(const Frame& frame) {
  // init first_encountered_overlap_map_
  first_encountered_overlap_map_.clear();
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();//init时计算
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;//type ,path_overlap
    }
  }
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());

  Observe(frame);//更新每条引导线对应的类型与范围，将其放入first_encountered_overlap_map_中
  //first_encountered_overlap_map_[overlap.first] = overlap.second;//type ,path_overlap
  ScenarioDispatch(frame);//选择场景
}


void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());
  ScenarioConfig::ScenarioType scenario_type;

  int history_points_len = 0;
  if (injector_->learning_based_data() &&
      injector_->learning_based_data()->GetLatestLearningDataFrame()) {
    history_points_len = injector_->learning_based_data()
                                  ->GetLatestLearningDataFrame()
                                  ->adc_trajectory_point_size();
  }
  if ((planning_config_.learning_mode() == PlanningConfig::E2E ||
       planning_config_.learning_mode() == PlanningConfig::E2E_TEST) &&
      history_points_len >= FLAGS_min_past_history_points_len) {
    scenario_type = ScenarioDispatchLearning();
  } else {
    scenario_type = ScenarioDispatchNonLearning(frame);//true
  }

  ADEBUG << "select scenario: "
         << ScenarioConfig::ScenarioType_Name(scenario_type);

  // update PlanningContext
  UpdatePlanningContext(frame, scenario_type);

  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchLearning() {
  ////////////////////////////////////////
  // learning model scenario
  ScenarioConfig::ScenarioType scenario_type =
      ScenarioConfig::LEARNING_MODEL_SAMPLE;
  return scenario_type;
}

bool ScenarioManager::JudgeReachTargetPoint(
  const common::VehicleState& car_position,
  const common::PointENU& target_point) {
  double distance_to_vehicle =
    (car_position.x() - target_point.x()) *
    (car_position.x() - target_point.x()) +
    (car_position.y() - target_point.y()) *
    (car_position.y() - target_point.y());
  return distance_to_vehicle < FLAGS_threshold_distance_for_destination;
}

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchNonLearning(
    const Frame& frame) {
  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;
  ////////////////////////////////////////
  // Pad Msg scenario
  scenario_type = SelectPadMsgScenario(frame);//根据驾驶员操作，选择场景

  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  const common::PointENU& target_point =
  frame.local_view().routing->routing_request().dead_end_info().target_point();////目标点坐标
  const common::VehicleState& car_position = frame.vehicle_state();
  if (scenario_type == default_scenario_type_) {//如果默认类型是lane_follow
    // check current_scenario (not switchable)
    switch (current_scenario_->scenario_type()) {//默认为lane_follow
      case ScenarioConfig::LANE_FOLLOW:
      case ScenarioConfig::PULL_OVER:
        break;
      case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
      case ScenarioConfig::EMERGENCY_PULL_OVER:
      case ScenarioConfig::PARK_AND_GO:
      case ScenarioConfig::STOP_SIGN_PROTECTED:
      case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
      case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      case ScenarioConfig::VALET_PARKING:
      case ScenarioConfig::DEADEND_TURNAROUND:
        // transfer dead_end to lane follow, should enhance transfer logic
        if (JudgeReachTargetPoint(car_position, target_point)) {//到达目标点后将场景类型改为lane_follow
          scenario_type = ScenarioConfig::LANE_FOLLOW;
          reach_target_pose_ = true;
        }
      case ScenarioConfig::YIELD_SIGN:
        // must continue until finish
        if (current_scenario_->GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE) {
          scenario_type = current_scenario_->scenario_type();
        }
        break;
      default:
        break;
    }
  }
  ////////////////////////////////////////
  // ParkAndGo / starting scenario
  if (scenario_type == default_scenario_type_) {//true
    if (FLAGS_enable_scenario_park_and_go && !reach_target_pose_) {//true,若是没有到达目标点
      scenario_type = SelectParkAndGoScenario(frame);//场景类型变为park and go
    }
  }
  //如果当前速度小于最大速度，且目标与汽车之间的距离大于25m，
  //且车无邻近车道或者临近车道lane是非驾驶车道时，则将其设置为park_and_go场景

  ////////////////////////////////////////
  // intersection scenarios十字路口场景
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectInterceptionScenario(frame);
  }
  //根据车头与交通灯，交通标注与路口之间的距离选择对应的场景

  ////////////////////////////////////////
  // pull-over scenario
  //当只有一条引导线且车头距离目标的距离大于10m小于50m时触发pull_over场景
  if (scenario_type == default_scenario_type_) {
    if (FLAGS_enable_scenario_pull_over) {//true
      scenario_type = SelectPullOverScenario(frame);
    }
  }

  ////////////////////////////////////////
  // VALET_PARKING scenario代客停车
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectValetParkingScenario(frame);
  }
  ////////////////////////////////////////
  // dead end
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectDeadEndScenario(frame);
  }
  ////////////////////////////////////////
  return scenario_type;
}

bool ScenarioManager::IsBareIntersectionScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::BARE_INTERSECTION_UNPROTECTED);
}

bool ScenarioManager::IsStopSignScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
          scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED);
}

bool ScenarioManager::IsTrafficLightScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN);
}

bool ScenarioManager::IsYieldSignScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::YIELD_SIGN);
}

void ScenarioManager::UpdatePlanningContext(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  // BareIntersection scenario
  //检查是否为BareIntersection
  UpdatePlanningContextBareIntersectionScenario(frame, scenario_type);

  // EmergencyStop scenario
  //检查是否为EmergencyStop
  UpdatePlanningContextEmergencyStopcenario(frame, scenario_type);

  // PullOver & EmergencyPullOver scenarios
  //先检查，若不是pull_over场景则判断pull_over点与终点距离是否大于30m，若是则清除clear
  UpdatePlanningContextPullOverScenario(frame, scenario_type);

  // StopSign scenario
  //首先判断是否为StopSign scenario
  //之后根据StopSign找到初始化时overlap中的stop_sign的区域，将其放入：
  // injector_->planning_context()->mutable_planning_status()->mutable_stop_sign()
  //->set_current_stop_sign_overlap_id(map_itr->second.object_id)中
  UpdatePlanningContextStopSignScenario(frame, scenario_type);

  // TrafficLight scenario
  //首先判断是否为TrafficLight scenario
  //之后获得引导线中的首个traffic_light区域
  //之后在引导线中寻找同一区域相差不超过2m的traffic_light区域，将其放入
  //：injector_->planning_context()
  //        ->mutable_planning_status()
  //        ->mutable_traffic_light()
  //        ->add_current_traffic_light_overlap_id
  UpdatePlanningContextTrafficLightScenario(frame, scenario_type);

  // YieldSign scenario
  //与traffic_light处理相同
  UpdatePlanningContextYieldSignScenario(frame, scenario_type);
}

// update: bare_intersection status in PlanningContext
void ScenarioManager::UpdatePlanningContextBareIntersectionScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* bare_intersection = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_bare_intersection();

  if (!IsBareIntersectionScenario(scenario_type)) {//检查scenario——type是否为十字路口类型
    bare_intersection->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered pnc_junction
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::PNC_JUNCTION);
  if (map_itr != first_encountered_overlap_map_.end()) {
    bare_intersection->set_current_pnc_junction_overlap_id(
        map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: emergency_stop status in PlanningContext
void ScenarioManager::UpdatePlanningContextEmergencyStopcenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* emergency_stop = injector_->planning_context()
                             ->mutable_planning_status()
                             ->mutable_emergency_stop();
  if (scenario_type != ScenarioConfig::EMERGENCY_STOP) {
    emergency_stop->Clear();
  }
}

// update: stop_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextStopSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsStopSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered stop_sign
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::STOP_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: traffic_light(s) status in PlanningContext
void ScenarioManager::UpdatePlanningContextTrafficLightScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsTrafficLightScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered traffic_light
  std::string current_traffic_light_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_traffic_light_overlap_id = map_itr->second.object_id;
  }

  if (current_traffic_light_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  // find all the traffic light at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered traffic_light["
             << traffic_light_overlap.object_id << "] start_s["
             << traffic_light_overlap.start_s << "]";
    }
  }
}

// update: yield_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextYieldSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsYieldSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered yield_sign
  std::string current_yield_sign_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::YIELD_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_yield_sign_overlap_id = map_itr->second.object_id;
  }

  if (current_yield_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  // find all the yield_sign at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& yield_sign_overlaps =
      reference_line_info.reference_line().map_path().yield_sign_overlaps();
  auto yield_sign_overlap_itr = std::find_if(
      yield_sign_overlaps.begin(), yield_sign_overlaps.end(),
      [&current_yield_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_yield_sign_overlap_id;
      });
  if (yield_sign_overlap_itr == yield_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_yield_sign_overlap_start_s =
      yield_sign_overlap_itr->start_s;
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    const double dist =
        yield_sign_overlap.start_s - current_yield_sign_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_yield_sign()
          ->add_current_yield_sign_overlap_id(yield_sign_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered yield_sign["
             << yield_sign_overlap.object_id << "] start_s["
             << yield_sign_overlap.start_s << "]";
    }
  }
}

// update: pull_over status in PlanningContext
void ScenarioManager::UpdatePlanningContextPullOverScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();
  if (scenario_type == ScenarioConfig::PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::PULL_OVER);
    pull_over->set_plan_pull_over_path(true);
    return;
  } else if (scenario_type == ScenarioConfig::EMERGENCY_PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::EMERGENCY_PULL_OVER);
    return;
  }

  pull_over->set_plan_pull_over_path(false);

  // check pull_over_status left behind
  // keep it if close to destination, to keep stop fence
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    const auto& routing = frame.local_view().routing;
    if (routing->routing_request().waypoint_size() >= 2) {
      // keep pull-over stop fence if destination not changed
      const auto& reference_line_info = frame.reference_line_info().front();
      const auto& reference_line = reference_line_info.reference_line();

      common::SLPoint dest_sl;
      const auto& routing_end =
          *(routing->routing_request().waypoint().rbegin());
      reference_line.XYToSL(routing_end.pose(), &dest_sl);

      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      static constexpr double kDestMaxDelta = 30.0;  // meter
      if (std::fabs(dest_sl.s() - pull_over_sl.s()) > kDestMaxDelta) {
        injector_->planning_context()
            ->mutable_planning_status()
            ->clear_pull_over();
      }
    }
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
