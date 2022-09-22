<center><span style="font-size:2rem;font-weight:bold;">红绿灯以及路口场景</span></center>

|  版本  |    日期    |  作者  |  备注  |
| :----: | :--------: | :----: | :----: |
| V1.0.0 | 2022.09.13 | 庞明慧 | 第一版 |

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 场景选择

场景选择在`scenario_manage.cc`中

* 首先根据引导线获取该引导线中所有的交通标志重叠区域
* 之后遍历这些`overlaps`
* 当signal，yield，stop_sign，区域在前时，优先选择这些交通标志区域
* 若是`pnc_junction`区域在前，则选择与其最近的交通标志区域
* 如果两者的距离大于`10m`，则选择最近的`overlaps`
* 反之如果两者的距离小于`10m`，则优先处理交通标志区域

相应代码如下：
```c++
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
```

## 交通标志场景判断

首先判断该交通标志是什么：三种类型：

* **stop_sign**
* **yield_sign**
* **Signal**

### stop_sign场景判断

* 首先计算车头的s值
* 之后计算车头与stop_sign之间的距离
* 如果两者之间的距离小于`4m`大于`0m`
* 且当前场景为lane_follow或者park_and_go或者pull_over则返回场景**STOP_SIGN_UNPROTECTED**
* 反之返回**lane_follow**场景

代码如下：
```C++
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
```

### yield_sign场景判断

* 首先计算车头的s值
* 之后计算车头与yield_sign之间的距离
* 如果两者之间的距离小于`10m`大于`0m`
* 且当前场景为lane_follow或者park_and_go或者pull_over则返回场景**YIELD_SIGN**
* 反之返回**lane_follow**场景

代码如下：
```c++
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
```

### traffic_light场景判断

* 首先根据配置config文件计算处理交通灯时的最大距离：`start_check_distance=30m`
* 之后根据引导线计算车头的距离
* 之后遍历引导线上所有的signal区域将处于当前signal `2m`内的signal 放入next_traffic_lights中
* 之后遍历next_traffic_lights：
  * 首先计算车头到达该signal_light的距离
  * 之后排除距离大于`30m`的signal
  * 根据该signal的id获得该signal的所有子signal::sub_signal
    * 遍历该signal的所有sub_signal，若是存在arrow_left类型则将left_turn_signal置为true(即该组红绿灯存在左转向灯)
  * 判断该signal的颜色，如果不是绿色则将red_light置为true，并退出(即该红绿灯是红色的)
* next_traffic_lights遍历结束
* 之后获取该signal处的lane的type，如果type为`hdmap::Lane::RIGHT_TURN`，则right_turn为true,如果type为`hdmap::Lane:LEFT_TURN`,则left_turn为true
* 之后进行场景判断：
  * 如果type为右转(right_turn)且此时为红灯，则计算车头与该signal的距离，如果距离小于5m，则选择场景为：**TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN**
  * 如果type为左转(left_turn)且不存在左转灯(!left_turn_signal)，则计算车头与该signal的距离，如果距离小于30m,则选择场景为：**TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN**
  * 其他情况则选择场景为：**TRAFFIC_LIGHT_PROTECTED**

代码如下：
```C++
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
```



## 路口场景BARE_INTERSECTION_UNPROTECTED判断

* 首先获取该pnc_junction区域内lane的type

* 若type是no_turn(即直行)，则返回**lane_follow**场景

* 若type时turn类型(left_turn，right_turn，U_turn)则继续判断：

  如果车头与`pnc_junction::start_s`之间的距离小于`25m`则返回场景**BARE_INTERSECTION_UNPROTECTED**

代码如下：
```C++
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
```

# 场景执行

## 路口场景BARE_INTERSECTION_UNPROTECTED

### stage列表

1. **BARE_INTERSECTION_UNPROTECTED_APPROACH**
2. **BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE**

### Stage1：BARE_INTERSECTION_UNPROTECTED_APPROACH

#### task列表：

- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤：

* 首先获取该场景的config
* 之后执行各个task生成轨迹
* 之后从planning_context中获得当前pnc_junction的id，若是不存在则该场景完成
* 之后再从reference_line中获得当前pnc_junction，若是不存在则该场景完成
* 计算pnc_junction到车头的距离，如果车头超出pnc_junction0.3m之外，则该stage完成
* 若是没有超出则设置巡航速度为6.7056m/s==>24km/h
* 之后设置路权（作用未知）
* 再执行一次ExecuteTaskOnReferenceline(因为巡航速度改变了)
* [check_clear函数](# check_clear函数)
* [if(scenario_config_.enable_explicit_stop())](# if(scenario_config_.enable_explicit_stop()函数)//实际为false

#### 代码：

##### 完整代码

```C++
Stage::StageStatus BareIntersectionUnprotectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const std::string pnc_junction_overlap_id =
      GetContext()->current_pnc_junction_overlap_id;
  if (pnc_junction_overlap_id.empty()) {
    return FinishScenario();
  }

  // get overlap along reference line
  PathOverlap* current_pnc_junction = scenario::util::GetOverlapOnReferenceLine(
      reference_line_info, pnc_junction_overlap_id,
      ReferenceLineInfo::PNC_JUNCTION);
  if (!current_pnc_junction) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_pnc_junction =
      current_pnc_junction->start_s - adc_front_edge_s;
  ADEBUG << "pnc_junction_overlap_id[" << pnc_junction_overlap_id
         << "] start_s[" << current_pnc_junction->start_s
         << "] distance_adc_to_pnc_junction[" << distance_adc_to_pnc_junction
         << "]";
  if (distance_adc_to_pnc_junction < -kPassStopLineBuffer) {//当车头超出pnc_junction0.3米时，该stage完成
    // passed stop line
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().SetCruiseSpeed(
      scenario_config_.approach_cruise_speed());//6.7056m/s

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(current_pnc_junction->start_s,
                                            false);

  plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "BareIntersectionUnprotectedStageApproach planning error";
  }

  std::vector<std::string> wait_for_obstacle_ids;
  bool clear = CheckClear(reference_line_info, &wait_for_obstacle_ids);//15m以内无obs则true

  if (scenario_config_.enable_explicit_stop()) {//false
    bool stop = false;
    static constexpr double kCheckClearDistance = 5.0;  // meter
    static constexpr double kStartWatchDistance = 2.0;  // meter
    if (distance_adc_to_pnc_junction <= kCheckClearDistance &&
        distance_adc_to_pnc_junction >= kStartWatchDistance && !clear) {
      stop = true;
    } else if (distance_adc_to_pnc_junction < kStartWatchDistance) {
      // creeping area
      auto* bare_intersection_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_bare_intersection();
      int clear_counter = bare_intersection_status->clear_counter();
      clear_counter = clear ? clear_counter + 1 : 0;

      if (clear_counter >= 5) {
        clear_counter = 0;  // reset
      } else {
        stop = true;
      }
      // use PlanningContext instead of static counter for multi-ADC
      bare_intersection_status->set_clear_counter(clear_counter);
    }

    if (stop) {
      // build stop decision
      ADEBUG << "BuildStopDecision: bare pnc_junction["
             << pnc_junction_overlap_id << "] start_s["
             << current_pnc_junction->start_s << "]";
      const std::string virtual_obstacle_id =
          "PNC_JUNCTION_" + current_pnc_junction->object_id;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, current_pnc_junction->start_s,
          scenario_config_.stop_distance(),
          StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
          "bare intersection", frame,
          &(frame->mutable_reference_line_info()->front()));
    }
  }

  return Stage::RUNNING;
}
```

##### check_clear函数

* 遍历所有障碍物
  * 跳过虚拟障碍物
  * 跳过静态障碍物
  * 若是存在障碍物满足以下其中一个条件：
    * 长度大于0
    * min_t大于0.1
    * min_s大小于15m
  * 则返回false
  * 否则返回true

代码：
```C++
bool BareIntersectionUnprotectedStageApproach::CheckClear(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::string>* wait_for_obstacle_ids) {
  // TODO(all): move to conf
  static constexpr double kConf_min_boundary_t = 6.0;        // second
  static constexpr double kConf_ignore_max_st_min_t = 0.1;   // second
  static constexpr double kConf_ignore_min_st_min_s = 15.0;  // meter

  bool all_far_away = true;
  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {//遍历所有障碍物
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->reference_line_st_boundary().min_t() < kConf_min_boundary_t) {//6s，只判断6s内出现的障碍物
      const double kepsilon = 1e-6;
      double obstacle_traveled_s =
          obstacle->reference_line_st_boundary().bottom_left_point().s() -
          obstacle->reference_line_st_boundary().bottom_right_point().s();
      ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
             << obstacle->reference_line_st_boundary().min_t()
             << "] obstacle_st_min_s["
             << obstacle->reference_line_st_boundary().min_s()
             << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

      // ignore the obstacle which is already on reference line and moving
      // along the direction of ADC
      if (obstacle_traveled_s < kepsilon &&
          obstacle->reference_line_st_boundary().min_t() <//0.1
              kConf_ignore_max_st_min_t &&
          obstacle->reference_line_st_boundary().min_s() >//15
              kConf_ignore_min_st_min_s) {
        continue;
      }

      wait_for_obstacle_ids->push_back(obstacle->Id());//15m内的obs
      all_far_away = false;
    }
  }
  return all_far_away;
}
```

##### if(scenario_config_.enable_explicit_stop())函数

* 如果车头与pnc之间的距离小于5m大于2m且clear==fales(即周围存在障碍物)，stop=true
* 反之如果两者之间的距离小于2m，则计算clean的周期。若是clean的周期大于5则stop=false
* 反之stop=true
* 根据stop值，选择是否构建stop_wall

**注：即当两者之间的距离小于2m时，查看引导线中的障碍物，若是有障碍物则则构建stop_wall,若是没有则等待5个周期后继续执行**

### Stage2:**BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE**

#### task列表：

- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先根据task规划轨迹：ExecuteTaskOnReferenceline
* 之后判断stage是否完成：
  * 首先检查引导线中是否存在pnc_junction
  * 若是存在pnc_junction：
    * 若车头不在pnc_junction前后1m范围内，则该stage完成
    * 反之车头在pnc_junction内，若车尾超出pnc_junction2m外，则stage完成
    * 其他情况stage未完成
  * 若是不存在pnc_junction，则stage完成

#### 代码

##### 完整代码：

```C++
Stage::StageStatus BareIntersectionUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::BARE_INTERSECTION_UNPROTECTED, config_,
      injector_->planning_context(), false);//车超出pnc_junction2m则完成
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

CheckDone函数：

```C++
bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const PlanningContext* context,
    const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status = context->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          context->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status = context->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 40.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();//车尾的位置
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;//车尾距离交通灯的距离
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;//大于40m时完成
  }//引导线没有pnc_junction区域

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {//检查车是否在junction 2m范围内
    return true;//不在则完成
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}
```

## STOP_SIGN场景

场景名称：**STOP_SIGN_UNPROTECTED**

### stage列表

1. **stage_type: STOP_SIGN_UNPROTECTED_PRE_STOP**

2. **stage_type: STOP_SIGN_UNPROTECTED_STOP**

3. **stage_type: STOP_SIGN_UNPROTECTED_CREEP**

4. **stage_type: STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE**

### Stage1: STOP_SIGN_UNPROTECTED_PRE_STOP

#### task列表

- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先获取该场景的config;
* 根据task列表规划轨迹：`ExecuteTaskOnReferenceLine`
* 得到该stop_sign的id；
* 根据该id从reference_line_info中获取stop_overlap；
* 如果在该引导线中不存在该stop_sign_overlap，则说明该场景结束，场景完成：FinishScenario()
* **计算`车头 - stop_start_s`的值，当该值大于`0.3m`时，说明 车头已经超过了`stop_line_starts` `0.3m`,此时表明该场景结束：`FinishStage()`；**
* 如果该值小于`0.3m`(即stop_sign仍在车头前面)，则判断车是否停车`(CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s))`：
  * **如果测试车辆速度小于`0.2m/s`且车头与stop_sign_starts之间的距离小于`2m`则说明测试车辆已经停车，且停车合规，因此该场景结束`FinishStage()`;**
  * 其他情况表明测试车辆没有停车或者停车距离不合规`(v > 0.2m/s || distance >2m)`，则场景继续。
* 将处于stop_sign前方`5m`内的障碍物放入`injector`中的`wait_for_obstacles`中。

流程图如下所示：



#### 代码

##### 完整代码

```C++
Stage::StageStatus StopSignUnprotectedStagePreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: PreStop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }
  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =//车头位置减去stop_sign的位置
      adc_front_edge_s - current_stop_sign_overlap->start_s;
      //车仍未通过stop_sign
  if (distance_adc_pass_stop_sign <= kPassStopLineBuffer) {//0.3.如果车在（stop_sign+0.3）前
    // not passed stop line, check valid stop
    //当车头没有通过stop_sign时，若是其速度大于0.2m/s则false
    //若是车头与stop_sign距离大于2m，则返回false
    if (CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s)) {
      return FinishStage();
    }
  } else {//已经通过了stop_line
    // passed stop line
    return FinishStage();
  }
  // PRE-STOP
  //两个上述false的情况：速度大于0.2m/s，或者距离大于2m
  const PathDecision& path_decision = reference_line_info.path_decision();
  auto& watch_vehicles = GetContext()->watch_vehicles;

  std::vector<std::string> watch_vehicle_ids;
  for (const auto& vehicle : watch_vehicles) {
    std::copy(vehicle.second.begin(), vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));

    // for debug string
    std::string associated_lane_id = vehicle.first;
    std::string s;
    for (const std::string& vehicle_id : vehicle.second) {
      s = s.empty() ? vehicle_id : s + "," + vehicle_id;
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }
  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  //将watch_vehicle_ids放入wait_for_obstacle_id中
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }
//遍历所有障碍物，将在stop_s前面5米没所有的障碍物id 放入watch_vehicles中
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*obstacle, &watch_vehicles);
  }
  return Stage::RUNNING;
}
```

`AddWatchVehicle`函数

```C++
int StopSignUnprotectedStagePreStop::AddWatchVehicle(
    const Obstacle& obstacle, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& perception_obstacle_id =
      std::to_string(perception_obstacle.id());
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);

  // check type
  //如果type是行人(pedestrian)的话就返回0
  if (obstacle_type != PerceptionObstacle::UNKNOWN &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::VEHICLE) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "]. skip";
    return 0;
  }

  const auto point =
      common::util::PointFactory::ToPointENU(perception_obstacle.position());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  LaneInfoConstPtr obstacle_lane;
  //在障碍物5m范围内，60度范围捏找到最近的lane，并计算该障碍点到该lane的s,l值
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString() << "; heading[" << perception_obstacle.theta()
           << "]";
    return -1;
  }
  if (!obstacle_lane->IsOnLane(common::util::PointFactory::ToVec2d(
          perception_obstacle.position()))) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "]: is off road. " << point.DebugString()
           << "; heading[" << perception_obstacle.theta() << "]";
    return -1;
  }

  // check obstacle is on an associate lane guarded by stop sign
  std::string obstable_lane_id = obstacle_lane.get()->id().id();
  auto assoc_lane_it = std::find_if(
      GetContext()->associated_lanes.begin(),//在scenario创建时的init函数中获得
      GetContext()->associated_lanes.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });
  if (assoc_lane_it == GetContext()->associated_lanes.end()) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "] lane_id[" << obstable_lane_id
           << "] not associated with current stop_sign. skip";
    return -1;
  }

  // check a valid stop for stop line of the stop_sign
  auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
      obstacle_lane.get()->id());
  if (over_lap_info == nullptr) {
    AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
    return -1;
  }
  const double stop_line_s = over_lap_info->lane_overlap_info().start_s();
  const double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  const double distance_to_stop_line = stop_line_s - obstacle_end_s;

  if (distance_to_stop_line >
      scenario_config_.watch_vehicle_max_valid_stop_distance()) {//5m
    ADEBUG << "obstacle_id[" << perception_obstacle_id << "] type["
           << obstacle_type_name << "] distance_to_stop_line["
           << distance_to_stop_line << "]; stop_line_s" << stop_line_s
           << "]; obstacle_end_s[" << obstacle_end_s
           << "] too far from stop line. skip";
    return -1;
  }

  // use a vector since motocycles/bicycles can be more than one
  std::vector<std::string> vehicles =
      (*watch_vehicles)[obstacle_lane->id().id()];
  if (std::find(vehicles.begin(), vehicles.end(), perception_obstacle_id) ==
      vehicles.end()) {
    ADEBUG << "AddWatchVehicle: lane[" << obstacle_lane->id().id()
           << "] obstacle_id[" << perception_obstacle_id << "]";
    (*watch_vehicles)[obstacle_lane->id().id()].push_back(
        perception_obstacle_id);
  }

  return 0;
}
```

### Stage2: STOP_SIGN_UNPROTECTED_STOP

#### task列表

- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先获取该场景的`config`;
* 根据task规划轨迹：`ExecuteTaskOnReferenceLine`；
* 获得该stop_sign的id；
* 根据该id在引导线中获取该stop_sign的stop_overlap；
* 如果该引导线中不存在该stop_sign则说明场景结束：`FinishScenario()`;
* 设置stop_starts的路权(暂时没有用到)：`reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);`
* **计算车头超出stop_sign的距离，若是超出距离大于`1m`，则该stage结束：`FinishStage()`；**
* 计算上一个stage结束到当前的时刻的时间，若是该时间段小于`1s` , 则表明该stage仍在运行，返回running；
* **获取上一个stage计算的watch_vehicles，若是不存在则stage结束：`FinishStage()`；**
* 移除重复的watch_vehicles；
* **若是等待时间大于8s且此时watch_vehicles的数量小于2(0或者1)，则此时stage结束`FinishStage()`；**
* 移除与stop_line之间的距离大于10m的watch_vehicles。

流程图如下所示：

#### 代码

##### 完整代码

```C++
Stage::StageStatus StopSignUnprotectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // refresh overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;//stop的起始s值
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  static constexpr double kPassStopLineBuffer = 1.0;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_start_s;
  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {//车头超过stop_sign超过1m
    return FinishStage();
  }

  // check on wait-time
  auto start_time = GetContext()->stop_start_time;//stop_start_time是上一步stage完成的时间
  const double wait_time = Clock::NowInSeconds() - start_time;
  ADEBUG << "stop_start_time[" << start_time << "] wait_time[" << wait_time
         << "]";
  if (wait_time < scenario_config_.stop_duration_sec()) {//1s
    return Stage::RUNNING;
  }

  // check on watch_vehicles
  auto& watch_vehicles = GetContext()->watch_vehicles;
  if (watch_vehicles.empty()) {
    return FinishStage();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (const auto& watch_vehicle : watch_vehicles) {
    std::copy(watch_vehicle.second.begin(), watch_vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));
    // for debug
    std::string s;
    for (const std::string& vehicle : watch_vehicle.second) {
      s = s.empty() ? vehicle : s + "," + vehicle;
    }
    const std::string& associated_lane_id = watch_vehicle.first;
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // remove duplicates (caused when same vehicle on mutiple lanes)
  watch_vehicle_ids.erase(
      unique(watch_vehicle_ids.begin(), watch_vehicle_ids.end()),
      watch_vehicle_ids.end());

  if (watch_vehicle_ids.empty()) {
    return FinishStage();
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  // check timeout while waiting for only one vehicle
  //如果等待的时间大于8s且wathch_vehicle的数量小于等于1
  if (wait_time > scenario_config_.stop_timeout_sec() &&//8s
      watch_vehicle_ids.size() <= 1) {
    return FinishStage();
  }

//移除与stop_line之间距离大于10m的障碍物
  const PathDecision& path_decision = reference_line_info.path_decision();
  RemoveWatchVehicle(path_decision, &watch_vehicles);

  return Stage::RUNNING;
}
```

##### `RemoveWatchVehicle`函数

```C++
int StopSignUnprotectedStageStop::RemoveWatchVehicle(
    const PathDecision& path_decision, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  for (auto& vehicle : *watch_vehicles) {
    // associated_lane/stop_sign info
    std::string associated_lane_id = vehicle.first;
    auto assoc_lane_it = std::find_if(
        GetContext()->associated_lanes.begin(),
        GetContext()->associated_lanes.end(),
        [&associated_lane_id](
            std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
          return assc_lane.first.get()->id().id() == associated_lane_id;
        });
    if (assoc_lane_it == GetContext()->associated_lanes.end()) {
      continue;
    }
    auto stop_sign_over_lap_info =
        assoc_lane_it->second.get()->GetObjectOverlapInfo(
            hdmap::MakeMapId(associated_lane_id));
    if (stop_sign_over_lap_info == nullptr) {
      AERROR << "can't find stop_sign_over_lap_info for id: "
             << associated_lane_id;
      continue;
    }
    const double stop_line_end_s =
        stop_sign_over_lap_info->lane_overlap_info().end_s();

    const auto lane =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(associated_lane_id));
    if (lane == nullptr) {
      continue;
    }
    auto stop_sign_point = lane.get()->GetSmoothPoint(stop_line_end_s);

    std::vector<std::string> remove_vehicles;
    auto& vehicles = vehicle.second;
    for (const auto& perception_obstacle_id : vehicles) {
      // watched-vehicle info
      const PerceptionObstacle* perception_obstacle =
          path_decision.FindPerceptionObstacle(perception_obstacle_id);
      if (!perception_obstacle) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id
               << "] not exist";
        remove_vehicles.push_back(perception_obstacle_id);
        continue;
      }

      PerceptionObstacle::Type obstacle_type = perception_obstacle->type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      auto obstacle_point = common::util::PointFactory::ToPointENU(
          perception_obstacle->position());

      double distance =
          common::util::DistanceXY(stop_sign_point, obstacle_point);
      ADEBUG << "obstacle_id[" << perception_obstacle_id << "] distance["
             << distance << "]";

      // TODO(all): move 10.0 to conf
      if (distance > 10.0) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id << "]";
        remove_vehicles.push_back(perception_obstacle_id);
      }
    }
    for (const auto& perception_obstacle_id : remove_vehicles) {
      ADEBUG << "ERASE obstacle_id[" << perception_obstacle_id << "]";
      vehicles.erase(
          std::remove(vehicles.begin(), vehicles.end(), perception_obstacle_id),
          vehicles.end());
    }
  }

  return 0;
}
```

### Stage3 : STOP_SIGN_UNPROTECTED_CREEP

#### task列表

- [x] **CREEP_DECIDER**
- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先获取该stage的config，如果该config中对creep并未使能，则stage结束`FinishStage`；

* 根据task列表规划轨迹：`ExecuteTaskOnReferenceLine`;

  **CREEP_DECIDER的作用是在stop_sign的`end_s+4m`的位置构建stop_wall**

* 从引导线中获取该stop_sign_overlap，若是不存在则场景完成`FinishScenario()`;

* 在stop_s的位置设置路权；

* 计算上一stage结束到当前时刻的时间；

* 计算creep_stop_s=stop_sign_s+2;

* 计算车头与creep_stop_s的距离；

* 如果车头已经超过creep_stop_s，则生成减速轨迹；

* 反之则检查creep是否完成：

  * 如果stop_sign_end_s+4m与车头之间的距离大于0.4m且等待时间小于10s时，creep未完成
  * **反之遍历所有动态障碍物，如果测试车辆前方15m内没有障碍物(障碍物长度大于0.1)且度过5个计算周期，则creep完成，该stage结束**

#### 代码

```C++
Stage::StageStatus StopSignUnprotectedStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s = current_stop_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();//10s
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));

  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s =
      stop_sign_end_s + task->FindCreepDistance(*frame, reference_line_info);//2.0
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info, stop_sign_end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

`GenerateFixedDistanceCreepProfile函数`

```C++
SpeedData SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
    const double distance, const double max_speed) {
  static constexpr double kConstDeceleration = -0.8;  // (~3sec to fully stop)
  static constexpr double kProceedingSpeed = 2.23;    // (5mph proceeding speed)
  const double proceeding_speed = std::fmin(max_speed, kProceedingSpeed);
  const double distance_to_start_deceleration =
      proceeding_speed * proceeding_speed / kConstDeceleration / 2;
  bool is_const_deceleration_mode = distance < distance_to_start_deceleration;

  double a = kConstDeceleration;
  double t = 0.0;
  double s = 0.0;
  double v = proceeding_speed;

  static constexpr double kDeltaT = 0.1;

  SpeedData speed_data;
  while (s < distance && v > 0) {
    if (is_const_deceleration_mode) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::max(0.0, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
    } else {
      speed_data.AppendSpeedPoint(s, t, v, 0.0, 0.0);
      t += kDeltaT;
      s += kDeltaT * v;
      if (distance - s < distance_to_start_deceleration)
        is_const_deceleration_mode = true;
    }
  }

  return speed_data;
}
```

`CheckCreepDone函数`

```C++
bool CreepDecider::CheckCreepDone(const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info,
                                  const double traffic_sign_overlap_end_s,
                                  const double wait_time_sec,
                                  const double timeout_sec) {
  const auto& creep_config = config_.creep_decider_config();
  bool creep_done = false;
  double creep_stop_s = traffic_sign_overlap_end_s +
                        FindCreepDistance(frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < creep_config.max_valid_stop_distance() ||//0.4
      wait_time_sec >= timeout_sec) {//等待时间大于10s
    bool all_far_away = true;
    //遍历所有障碍物
    for (auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {//忽略虚拟障碍物与静态障碍物
        continue;
      }
      if (obstacle->reference_line_st_boundary().min_t() <
          creep_config.min_boundary_t()) {//6
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() <
                creep_config.ignore_max_st_min_t() &&//0.1
            obstacle->reference_line_st_boundary().min_s() >
                creep_config.ignore_min_st_min_s()) {//15
          continue;
        }
        all_far_away = false;
        break;
      }//小于6s obs处理结束
    }//障碍物遍历结束

    auto* creep_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_creep_decider();
    int creep_clear_counter = creep_decider_status->creep_clear_counter();
    creep_clear_counter = all_far_away ? creep_clear_counter + 1 : 0;
    if (creep_clear_counter >= 5) {
      creep_clear_counter = 0;  // reset
      creep_done = true;
    }
    // use PlanningContext instead of static counter for multi-ADC
    creep_decider_status->set_creep_clear_counter(creep_clear_counter);
  }//if结束

  return creep_done;
}
```

### Stage4: STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE

#### task列表

- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先根据task规划轨迹；
* 之后判断stage是否结束，如果结束则stage完成:
  * 首先判断该引导线处是否存在`pnc_junction`区域，若是存在则检查测试车辆是否在junction 2m范围内，若不在则stage完成，否则未完成；
  * 若是不存在pnc_junction区域，则检查车头是佛超过stop_sign区域40m外，若是则stage完成，否则未完成。

#### 代码

##### 完整代码

```C++
Stage::StageStatus StopSignUnprotectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageIntersectionCruise plan error";
  }

  bool stage_done =
      stage_impl_.CheckDone(*frame, ScenarioConfig::STOP_SIGN_UNPROTECTED,
                            config_, injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

##### CheckDone函数

```C++
bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const PlanningContext* context,
    const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status = context->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          context->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status = context->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 40.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();//车尾的位置
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;//车尾距离交通灯的距离
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;//大于40m时完成
  }//引导线没有pnc_junction区域

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {//检查车是否在junction 2m范围内
    return true;//不在则完成
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}
```

## Yield_sign场景

### stage列表

1. **YIELD_SIGN_APPROACH**
2. **YIELD_SIGN_CREEP**

### Stage1: YIELD_SIGN_APPROACH

#### task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

#### 执行步骤

* 首先根据task列表规划轨迹；
* 之后根据yiled_id从引导线中获取overlap,若overlaps不存在则场景完成：`FinishScenario()`；
* 计算车头超出yield_sign的距离，若是距离大于`0.3m`则说明该stage完成；
* 否则判断车头与yield_sign之间的距离，若是距离大于`4.5m`则stage继续；
* 若是距离小于`4.5m`，则遍历引导线中的障碍物，若是在测试车辆前方`15m`内不存在动态障碍物，则stage完成。

#### 代码

##### 完整代码

```C++
Stage::StageStatus YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  if (GetContext()->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  for (const auto& yield_sign_overlap_id :
       GetContext()->current_yield_sign_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            reference_line_info, yield_sign_overlap_id,
            ReferenceLineInfo::YIELD_SIGN);
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);

    static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
           << current_yield_sign_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "]";
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {//4.5
      // close enough, check yield_sign clear
      yield_sign_done = true;
      const auto& path_decision = reference_line_info.path_decision();
      for (const auto* obstacle : path_decision.obstacles().Items()) {
        const std::string& obstacle_id = obstacle->Id();
        std::string obstacle_type_name =
            PerceptionObstacle_Type_Name(obstacle->Perception().type());
        ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
               << obstacle_id << "] type[" << obstacle_type_name << "]";
        if (obstacle->IsVirtual()) {
          continue;
        }

        if (obstacle->reference_line_st_boundary().IsEmpty()) {
          continue;
        }

        static constexpr double kMinSTBoundaryT = 6.0;  // sec
        if (obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
          continue;
        }
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        // max st_min_t(sec) to ignore
        static constexpr double kIgnoreMaxSTMinT = 0.1;
        // min st_min_s(m) to ignore
        static constexpr double kIgnoreMinSTMinS = 15.0;
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT &&
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) {
          continue;
        }

        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_yield_sign()
            ->add_wait_for_obstacle_id(obstacle->Id());

        yield_sign_done = false;
      }
    }

    if (yield_sign_done) {
      return FinishStage();
    }
  }

  return Stage::RUNNING;
}
```

### Stage2 :YIELD_SIGN_CREEP

#### task列表

- [x] **CREEP_DECIDER**
- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

#### 执行步骤

* 首先根据task列表规划轨迹；
* **之后获取yield_sign的id并从引导线中获得overlap区域，若是引导线中不存在则该场景结束；**
* 计算上一个stage完成到目前时刻的时间；
* 计算creep_stop_s = yield_s + 2;
* 计算车头与yield_sign之间的距离；
* 若是距离小于0.0，则生成减速轨迹；
* 否则检查creep是否结束：
  * 计算creep_stop_s与车头之间的距离
  * 若是距离大于`0.4m`且等待时间小于10s,则stage未完成；
  * **反之，遍历所有的动态障碍物，若是测试车辆前方15m内没有障碍物则stage结束。**

#### 代码

##### 完整代码

```C++
Stage::StageStatus YieldSignStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "YieldSignStageCreep planning error";
  }

  if (GetContext()->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string yield_sign_overlap_id =
      GetContext()->current_yield_sign_overlap_ids[0];

  // get overlap along reference line
  PathOverlap* current_yield_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                yield_sign_overlap_id,
                                                ReferenceLineInfo::YIELD_SIGN);
  if (!current_yield_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double yield_sign_start_s = current_yield_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(yield_sign_start_s, false);

  const double yield_sign_end_s = current_yield_sign_overlap->end_s;
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();//10s
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));

  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s =
      yield_sign_end_s + task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info, yield_sign_end_s,
                           wait_time, timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

##### `CheckCreepDone函数`

```C++
bool CreepDecider::CheckCreepDone(const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info,
                                  const double traffic_sign_overlap_end_s,
                                  const double wait_time_sec,
                                  const double timeout_sec) {
  const auto& creep_config = config_.creep_decider_config();
  bool creep_done = false;
  double creep_stop_s = traffic_sign_overlap_end_s +
                        FindCreepDistance(frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < creep_config.max_valid_stop_distance() ||//0.4
      wait_time_sec >= timeout_sec) {//等待时间大于10s
    bool all_far_away = true;
    //遍历所有障碍物
    for (auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {//忽略虚拟障碍物与静态障碍物
        continue;
      }
      if (obstacle->reference_line_st_boundary().min_t() <
          creep_config.min_boundary_t()) {//6
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() <
                creep_config.ignore_max_st_min_t() &&//0.1
            obstacle->reference_line_st_boundary().min_s() >
                creep_config.ignore_min_st_min_s()) {//15
          continue;
        }
        all_far_away = false;
        break;
      }//小于6s obs处理结束
    }//障碍物遍历结束

    auto* creep_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_creep_decider();
    int creep_clear_counter = creep_decider_status->creep_clear_counter();
    creep_clear_counter = all_far_away ? creep_clear_counter + 1 : 0;
    if (creep_clear_counter >= 5) {
      creep_clear_counter = 0;  // reset
      creep_done = true;
    }
    // use PlanningContext instead of static counter for multi-ADC
    creep_decider_status->set_creep_clear_counter(creep_clear_counter);
  }//if结束

  return creep_done;
}
```

## traffic_light场景

### Traffic_light_unprotected_right_trun场景

#### Stage列表

1. **Traffic_light_unprotected_right_turn_stop**
2. **Traffic_light_unprotected_right_creep**
3. **Traffic_light_unprotected_right_trun_intersection_cruise**

#### Stage1: **Traffic_light_unprotected_right_turn_stop**

##### task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 首先根据task规划轨迹；
* 遍历该场景中的traffic_light
  * 计算车头到stop_line_s的距离;
  * 获得该tarffic_light的color;
  * 如果两者之间距离大于2m则traffic_light_all_stop=true,并跳出循环；反之继续；
  * 如果该traffic_light的color不是绿色，且不存在右转红灯，则traffic_light_all_green=false，并跳出循环；
* 如果车头与stop_s之间距离小于2m且是绿灯，则该场景完成，下一个stage为stage3
* 如果该traffic_light存在右转红灯，且此时车头与stop_s距离小于2m，且不是绿灯时，判断：
  * 计算车头超出stop_s的距离，若是该距离大于3m，则该stage完成；
    * 若是此时测试车辆速度大于3m/s，则下一个stage是stage3,否则下一个stage是stage2;

**总结：**

**若是车头与stop_s之间的距离小于2m,且此时为绿灯则将该stage切换为stage3;**

**若此时该traffic_light存在右转红灯，且不是绿灯时，则判断车头超出stop_s的距离，若是距离大于3m，则该stage完成；**
**此时判断测试车辆的速度，若是速度大于3m/s则下一个stage为stage3反之下一个stage为stage2。**

##### 代码

```C++
Stage::StageStatus TrafficLightUnprotectedRightTurnStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightRightTurnUnprotectedStop planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  bool traffic_light_all_stop = true;
  bool traffic_light_all_green = true;
  bool traffic_light_no_right_turn_on_red = false;
  PathOverlap* current_traffic_light_overlap = nullptr;

  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    current_traffic_light_overlap = scenario::util::GetOverlapOnReferenceLine(
        reference_line_info, traffic_light_overlap_id,
        ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    // check distance to stop line
    if (distance_adc_to_stop_line >
        scenario_config_.max_valid_stop_distance()) {//2
      traffic_light_all_stop = false;
      break;
    }

    // check on traffic light color
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_green = false;
      traffic_light_no_right_turn_on_red =
          CheckTrafficLightNoRightTurnOnRed(traffic_light_overlap_id);//该signal无右转红灯为true
      break;
    }
  }//范围内交通灯区域遍历结束

  if (traffic_light_all_stop && traffic_light_all_green) {//如果车头距离小于2m且为绿灯，则
    return FinishStage(true);
  }

  if (!traffic_light_no_right_turn_on_red) {//traffic_light有右转红灯
    if (traffic_light_all_stop && !traffic_light_all_green) {//距离小于2m且不是绿灯
      // check distance pass stop line
      const double distance_adc_pass_stop_line =
          adc_front_edge_s - current_traffic_light_overlap->end_s;//车头超出的距离
      ADEBUG << "distance_adc_pass_stop_line[" << distance_adc_pass_stop_line
             << "]";
      if (distance_adc_pass_stop_line >
          scenario_config_.min_pass_s_distance()) {//3
        return FinishStage(false);
      }

      if (scenario_config_.enable_right_turn_on_red()) {//false
        // when right_turn_on_red is enabled
        // check on wait-time
        if (GetContext()->stop_start_time == 0.0) {
          GetContext()->stop_start_time = Clock::NowInSeconds();
        } else {
          auto start_time = GetContext()->stop_start_time;
          const double wait_time = Clock::NowInSeconds() - start_time;
          ADEBUG << "stop_start_time[" << start_time << "] wait_time["
                 << wait_time << "]";
          if (wait_time >
              scenario_config_.red_light_right_turn_stop_duration_sec()) {
            return FinishStage(false);
          }
        }
      }//false结束
    }
  }

  return Stage::RUNNING;
}
```

#### Stage2: **Traffic_light_unprotected_right_creep**

##### task列表

- [x] **CREEP_DECIDER**
- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

##### 执行步骤

* 首先根据task规划轨迹；
* 计算等待时间：上一时刻完成到当前时刻的时间
* 计算creep_stop_s:stop_s+2
* 若是车头与creep_stop_s之间的距离小于0.0，则生成减速轨迹；
* 检查creep是否结束：
  * 计算creep_stop_s = stop_s+2;
  * 计算车头到creep_stop_s之间的距离；
  * 如果距离大于2m且等待时间小于10s，则返回false;
  * 反之遍历所有的动态障碍物，若是测试车辆前方15m内不存在动态障碍物且等待5个计算周期，则creep结束。

##### 代码

```C++
Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      GetContext()->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                traffic_light_overlap_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();//10s
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));
  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s = current_traffic_light_overlap->end_s +
                        task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info,
                           current_traffic_light_overlap->end_s, wait_time,
                           timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

CheckCreepDone函数

```C++
bool CreepDecider::CheckCreepDone(const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info,
                                  const double traffic_sign_overlap_end_s,
                                  const double wait_time_sec,
                                  const double timeout_sec) {
  const auto& creep_config = config_.creep_decider_config();
  bool creep_done = false;
  double creep_stop_s = traffic_sign_overlap_end_s +
                        FindCreepDistance(frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < creep_config.max_valid_stop_distance() ||//0.4
      wait_time_sec >= timeout_sec) {//等待时间大于10s
    bool all_far_away = true;
    //遍历所有障碍物
    for (auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {//忽略虚拟障碍物与静态障碍物
        continue;
      }
      if (obstacle->reference_line_st_boundary().min_t() <
          creep_config.min_boundary_t()) {//6
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() <
                creep_config.ignore_max_st_min_t() &&//0.1
            obstacle->reference_line_st_boundary().min_s() >
                creep_config.ignore_min_st_min_s()) {//15
          continue;
        }
        all_far_away = false;
        break;
      }//小于6s obs处理结束
    }//障碍物遍历结束

    auto* creep_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_creep_decider();
    int creep_clear_counter = creep_decider_status->creep_clear_counter();
    creep_clear_counter = all_far_away ? creep_clear_counter + 1 : 0;
    if (creep_clear_counter >= 5) {
      creep_clear_counter = 0;  // reset
      creep_done = true;
    }
    // use PlanningContext instead of static counter for multi-ADC
    creep_decider_status->set_creep_clear_counter(creep_clear_counter);
  }//if结束

  return creep_done;
}
```

#### Stage3: Traffic_light_unprotected_right_trun_intersection_cruise

##### task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 首先根据task规划轨迹
* 之后检查是否完成：
  * 若是引导线中午pnc_junction,则判断车尾与traffic_lighe之间的距离是否大于40m,大于则完成。
  * 若是不存在pnc_junction,则判断车尾与traffic_light值的距离是否大于2m,大于则为true,反之false。

##### 代码：

```C++
TrafficLightUnprotectedRightTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN, config_,
      injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

CheckDone函数

```C++
bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const PlanningContext* context,
    const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status = context->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          context->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status = context->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 40.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();//车尾的位置
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;//车尾距离交通灯的距离
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;//大于40m时完成
  }//引导线没有pnc_junction区域

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {//检查车是否在junction 2m范围内
    return true;//不在则完成
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}
```

### Traffic_light_unprotected_left_turn场景

#### Stage列表

1. **Traffic_light_unprotected_letft_turn_approach**
2. **Traffic_light_unprotected_left_turn_creep**
3. **Traffic_light_unprotected_left_turn_intersection_cruise**

#### Stage1: Traffic_light_unprotected_letft_turn_approach

##### task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 设置6m/s的巡航速度

* 根据task规划轨迹

* 遍历该traffic的所有light
  * 得到该traffic_light的color
  
  * 如果车头已经超出该traffic_light则该场景结束；
  
  * 如果该traffic_light的color颜色不是绿色的，且车头与stop_line_s距离大于2m;
  
    则跳出循环，赋值traffic_light_all_done = false
  
* 如果不存在traffic_light则该场景完成

* 如果traffic_light_all_done ==true，则该stage完成:若是速度大于5.56m/s则下一个stage为stage3,否则下一个stage为stage2

##### 代码

```C++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().SetCruiseSpeed(
      scenario_config_.approach_cruise_speed());//6m/s

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageApproach planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();//车头位置

  PathOverlap* traffic_light = nullptr;
  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {//遍历当前所有light
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                  traffic_light_overlap_id,
                                                  ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    traffic_light = current_traffic_light_overlap;

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
           << "] start_s[" << current_traffic_light_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "] color[" << signal_color << "]";

    if (distance_adc_to_stop_line < 0)//已经超出该signal范围
        return FinishStage(frame);
    // check on traffic light color and distance to stop line
    //如果不是绿灯且距离停车线仍大于2m则红绿灯场景未完成，退出
    if (signal_color != TrafficLight::GREEN ||
        distance_adc_to_stop_line >=
            scenario_config_.max_valid_stop_distance()) {//2m
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light == nullptr) {
    return FinishScenario();
  }

  if (traffic_light_all_done) {
    return FinishStage(frame);
  }

  return Stage::RUNNING;
}
```

FinishStage函数

```C++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageApproach::FinishStage(
    Frame* frame) {
  // check speed at stop_stage
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > scenario_config_.max_adc_speed_before_creep()) {
    // skip creep
    next_stage_ = ScenarioConfig ::
        TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE;
  } else {
    // creep
    // update PlanningContext
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->mutable_done_traffic_light_overlap_id()
        ->Clear();
    for (const auto& traffic_light_overlap_id :
         GetContext()->current_traffic_light_overlap_ids) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
    }

    GetContext()->creep_start_time = Clock::NowInSeconds();
    next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP;
  }

  // reset cruise_speed
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  reference_line_info.SetCruiseSpeed(FLAGS_default_cruise_speed);

  return Stage::FINISHED;
}
```

#### Stage2 :**Traffic_light_unprotected_left_turn_creep**

##### task列表

- [x] **CREEP_DECIDER**
- [x] **path_lane_borrow_decider**
- [x] **path_bound_decider**
- [x] **piecewise_jerk_path_optimizer**
- [x] **path_assessment_decider**
- [x] **rule_based_stop_decider**
- [x] **st_bounds_decider**
- [x] **speed_bounds_priori_decider**
- [x] **speed_heuristic_optimizer**
- [x] **speed_decider**
- [x] **speed_bounds_final_decider**
- [x] **piecewise_jerk_nonlinear_speed_optimizer**

##### 执行步骤

* 首先根据task规划轨迹
* 若是当前traffic_light_overlap为空，则场景结束
* 计算等待时间
* 计算creep_stop_s
* 若是车头超出creep_stop_s，则生成减速轨迹
* 之后检查creep_done：
  若是车头与creep_stop_s之间的距离小于2m且测试车辆前方15m范围内不存在障碍物，则返回true.

##### 代码

```C++
Stage::StageStatus TrafficLightUnprotectedLeftTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      GetContext()->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                traffic_light_overlap_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));
  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s = current_traffic_light_overlap->end_s +
                        task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info,
                           current_traffic_light_overlap->end_s, wait_time,
                           timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}
```

CheckCreepDone函数

```C++
bool CreepDecider::CheckCreepDone(const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info,
                                  const double traffic_sign_overlap_end_s,
                                  const double wait_time_sec,
                                  const double timeout_sec) {
  const auto& creep_config = config_.creep_decider_config();
  bool creep_done = false;
  double creep_stop_s = traffic_sign_overlap_end_s +
                        FindCreepDistance(frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < creep_config.max_valid_stop_distance() ||//0.4
      wait_time_sec >= timeout_sec) {//等待时间大于10s
    bool all_far_away = true;
    //遍历所有障碍物
    for (auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {//忽略虚拟障碍物与静态障碍物
        continue;
      }
      if (obstacle->reference_line_st_boundary().min_t() <
          creep_config.min_boundary_t()) {//6
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() <
                creep_config.ignore_max_st_min_t() &&//0.1
            obstacle->reference_line_st_boundary().min_s() >
                creep_config.ignore_min_st_min_s()) {//15
          continue;
        }
        all_far_away = false;
        break;
      }//小于6s obs处理结束
    }//障碍物遍历结束

    auto* creep_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_creep_decider();
    int creep_clear_counter = creep_decider_status->creep_clear_counter();
    creep_clear_counter = all_far_away ? creep_clear_counter + 1 : 0;
    if (creep_clear_counter >= 5) {
      creep_clear_counter = 0;  // reset
      creep_done = true;
    }
    // use PlanningContext instead of static counter for multi-ADC
    creep_decider_status->set_creep_clear_counter(creep_clear_counter);
  }//if结束

  return creep_done;
}
```

#### Stage3 : Traffic_light_unprotected_left_turn_intersection_cruise

##### task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 根据task规划代码
* 检查stage是否结束：
  * 若是引导线上存在pnc_junction,则车2m范围内不存在pnc_junction时，stage结束
  * 若是引导线上不存在pnc_junction,则车尾超出交通灯overlap40m时，stage结束

##### 代码

```C++
TrafficLightUnprotectedLeftTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = stage_impl_.CheckDone(
      *frame, ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN, config_,
      injector_->planning_context(), true);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

CheckDone函数

```C++
bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const PlanningContext* context,
    const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status = context->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          context->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status = context->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 40.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();//车尾的位置
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;//车尾距离交通灯的距离
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;//大于40m时完成
  }//引导线没有pnc_junction区域

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {//检查车是否在junction 2m范围内
    return true;//不在则完成
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}
```

### Traffic_light_protected_scenario场景

#### stage列表

1. **traffic_light_protected_approach**
2. **traffic_light_protected_intersection_cruise**

#### Stage1 : Traffic_light_protected_approach

##### Task列表

- [x] **PATH_LANE_BORROW_DECIDER**

- [x] **PATH_BOUNDS_DECIDER**

- [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

- [x] **PATH_ASSESSMENT_DECIDER**

- [x] **PATH_DECIDER**

- [x] **RULE_BASED_STOP_DECIDER**

- [x] **ST_BOUNDS_DECIDER**

- [x] **SPEED_BOUNDS_PRIORI_DECIDER**

- [x] **SPEED_HEURISTIC_OPTIMIZER**

- [x] **SPEED_DECIDER**

- [x] **SPEED_BOUNDS_FINAL_DECIDER**

- [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 首先根据task列表规划轨迹

* 遍历当前所有的traffic_light:

  * 计算车头与当前traffic_light的距离
  * 得到当前traffic_light的color
  * 如果距离大于2m,则跳出循环，stage未完成
  * 如果color不是绿色的，则跳出循环，stage未完成
  * 其他情况stage完成

  ##### 代码

  ```C++
  Stage::StageStatus TrafficLightProtectedStageApproach::Process(
      const TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: Approach";
    CHECK_NOTNULL(frame);
  
    scenario_config_.CopyFrom(GetContext()->scenario_config);
  
    bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (!plan_ok) {
      AERROR << "TrafficLightProtectedStageApproach planning error";
    }
  
    if (GetContext()->current_traffic_light_overlap_ids.empty()) {
      return FinishScenario();
    }
  
    const auto& reference_line_info = frame->reference_line_info().front();
  
    bool traffic_light_all_done = true;
    for (const auto& traffic_light_overlap_id :
         GetContext()->current_traffic_light_overlap_ids) {
      // get overlap along reference line
      PathOverlap* current_traffic_light_overlap =
          scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                    traffic_light_overlap_id,
                                                    ReferenceLineInfo::SIGNAL);
      if (!current_traffic_light_overlap) {
        continue;
      }
  
      // set right_of_way_status
      reference_line_info.SetJunctionRightOfWay(
          current_traffic_light_overlap->start_s, false);
  
      const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
      const double distance_adc_to_stop_line =
          current_traffic_light_overlap->start_s - adc_front_edge_s;
      auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
      ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap_id
             << "] start_s[" << current_traffic_light_overlap->start_s
             << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
             << "] color[" << signal_color << "]";
  
      // check distance to stop line
      //如果车距离停车线大于2m则没完成
      if (distance_adc_to_stop_line >
          scenario_config_.max_valid_stop_distance()) {//2
        traffic_light_all_done = false;
        break;
      }
  //如果不是绿灯则没完成
      // check on traffic light color
      if (signal_color != TrafficLight::GREEN) {
        traffic_light_all_done = false;
        break;
      }
    }//over_lap遍历结束
  
    if (traffic_light_all_done) {
      return FinishStage();
    }
  
    return Stage::RUNNING;
  }
  ```

  #### Stage2 ： traffic_light_protected_intersection_cruise

  ##### Task列表

  - [x] **PATH_LANE_BORROW_DECIDER**

  - [x] **PATH_BOUNDS_DECIDER**

  - [x] **PIECEWISE_JERK_PATH_OPTIMIZER**

  - [x] **PATH_ASSESSMENT_DECIDER**

  - [x] **PATH_DECIDER**

  - [x] **RULE_BASED_STOP_DECIDER**

  - [x] **ST_BOUNDS_DECIDER**

  - [x] **SPEED_BOUNDS_PRIORI_DECIDER**

  - [x] **SPEED_HEURISTIC_OPTIMIZER**

  - [x] **SPEED_DECIDER**

  - [x] **SPEED_BOUNDS_FINAL_DECIDER**

  - [x] **PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER**

##### 执行步骤

* 首先根据task规划轨迹
* 检查stage是否完成：
  * 若是引导线上存在pnc_junction,则车2m范围内不存在pnc_junction时，stage结束
  * 若是引导线上不存在pnc_junction,则车尾超出交通灯overlap40m时，stage结束

##### 代码

```C++
Stage::StageStatus TrafficLightProtectedStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStageIntersectionCruise plan error";
  }

  bool stage_done =
      stage_impl_.CheckDone(*frame, ScenarioConfig::TRAFFIC_LIGHT_PROTECTED,
                            config_, injector_->planning_context(), true);
  if (stage_done) {
    return FinishStage();
  }
  return Stage::RUNNING;
}
```

CheckDone函数

```C++
bool StageIntersectionCruiseImpl::CheckDone(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type,
    const ScenarioConfig::StageConfig& config, const PlanningContext* context,
    const bool right_of_way_status) {
  const auto& reference_line_info = frame.reference_line_info().front();

  const auto& pnc_junction_overlaps =
      reference_line_info.reference_line().map_path().pnc_junction_overlaps();
  if (pnc_junction_overlaps.empty()) {
    // TODO(all): remove when pnc_junction completely available on map
    // pnc_junction not exist on map, use current traffic_sign's end_s
    // get traffic sign overlap along reference line
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    if (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
        scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED) {
      // stop_sign scenarios
      const auto& stop_sign_status = context->planning_status().stop_sign();
      const std::string traffic_sign_overlap_id =
          stop_sign_status.current_stop_sign_overlap_id();
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::STOP_SIGN);
    } else if (scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
               scenario_type ==
                   ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
      // traffic_light scenarios
      const auto& traffic_light_status =
          context->planning_status().traffic_light();
      const std::string traffic_sign_overlap_id =
          traffic_light_status.current_traffic_light_overlap_id_size() > 0
              ? traffic_light_status.current_traffic_light_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::SIGNAL);
    } else if (scenario_type == ScenarioConfig::YIELD_SIGN) {
      // yield_sign scenarios
      const auto& yield_sign_status = context->planning_status().yield_sign();
      const std::string traffic_sign_overlap_id =
          yield_sign_status.current_yield_sign_overlap_id_size() > 0
              ? yield_sign_status.current_yield_sign_overlap_id(0)
              : "";
      traffic_sign_overlap = scenario::util::GetOverlapOnReferenceLine(
          reference_line_info, traffic_sign_overlap_id,
          ReferenceLineInfo::YIELD_SIGN);
    }

    if (!traffic_sign_overlap) {
      return true;
    }

    static constexpr double kIntersectionPassDist = 40.0;  // unit: m
    const double adc_back_edge_s =
        reference_line_info.AdcSlBoundary().start_s();//车尾的位置
    const double distance_adc_pass_traffic_sign =
        adc_back_edge_s - traffic_sign_overlap->end_s;//车尾距离交通灯的距离
    ADEBUG << "distance_adc_pass_traffic_sign["
           << distance_adc_pass_traffic_sign << "] traffic_sign_end_s["
           << traffic_sign_overlap->end_s << "]";

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(traffic_sign_overlap->start_s,
                                              right_of_way_status);

    return distance_adc_pass_traffic_sign >= kIntersectionPassDist;//大于40m时完成
  }//引导线没有pnc_junction区域

  if (!planning::util::CheckInsidePnCJunction(reference_line_info)) {//检查车是否在junction 2m范围内
    return true;//不在则完成
  }

  // set right_of_way_status
  hdmap::PathOverlap pnc_junction_overlap;
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  reference_line_info.SetJunctionRightOfWay(pnc_junction_overlap.start_s,
                                            right_of_way_status);

  return false;
}
```

