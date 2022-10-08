<center><span style="font-size:2rem;font-weight:bold;">Speed_Bound_decider解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

speed_bound_decider分为两个task:

* SPEED_BOUNDS_PRIORI_DECIDER

* SPEED_BOUNDS_FINAL_DECIDER

SPEED_BOUNDS_PRIORI_DECIDER是在st_bound_decider结束之后，根据障碍物已有的决策信息调整更新障碍物的boundary，为DP搜索做准备。

SPEED_BOUNDS_FINAL_DECIDER是在DP搜索以及speed_decider对所有障碍物作出决策后，在根据决策信息对障碍物的boundary进行调整，为后面的优化做准备。

两者的不同：

两者的区别有两点：

* SPEED_BOUNDS_PRIORI_DECIDER执行时，当时大多数的障碍物没有纵向决策，因此对障碍物的st_boundary更新并不重要，而SPEED_BOUNDS_FINAL_DECIDER执行时，由于已经执行过speed_decider因此会根据决策调整st_boundary
* 两者的第二个区别是在创建stop_obstacle的st_boundary时会对s_max加一个buff值，该值数值不同，但不影响全局

# 函数处理流程

## 首先根据path等信息实例化STBoundaryMapper类(主要为类内赋值操作)

```C++
  STBoundaryMapper boundary_mapper(
      speed_bounds_config_, reference_line, path_data,
      path_data.discretized_path().Length(), speed_bounds_config_.total_time(),
      injector_);
```

## 之后计算障碍物的st_boundary

```C++
  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==//计算障碍物的boundary
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

### **ComputeSTBoundary函数流程**

* 遍历所有的障碍物
* 如果该障碍物没有纵向决策，则计算其的st_boundary，之后continue
  如果使用st_bound_decider则使用该task生成的boundary

```C++
    //如果该障碍物之前没有纵向决策（则计算该障碍物的stboundary）
    //如果使用stbound则跳过
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }
```

```c++
void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  if (FLAGS_use_st_drivable_boundary) {//true
    return;
  }
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  obstacle->set_path_st_boundary(boundary);
}
```

* 如果该障碍物存在纵向决策，则根据决策信息调整其st_boundar
  过程：
  * 首先找到最近的停止决策障碍物
  * 如果决策是follow、overtake、yield，则根据决策更新障碍物的st_bounday
  * 如果是其他决策，跳过

```C++
    const auto& decision = ptr_obstacle->LongitudinalDecision();
    if (decision.has_stop()) {//停止决策，找到最近的停止s值
      // 1. Store the closest stop fence info.
      // TODO(all): store ref. s value in stop decision; refine the code then.
      common::SLPoint stop_sl_point;
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      const double stop_s = stop_sl_point.s();

      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
```

**ComputeSTBoundaryWithDecision函数**

* 首先更新该障碍物的lower_points与up_points,并更新其boundary

```C++
  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {//当给该障碍物添加过st_boundary时，该值为true
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
```

```C++
STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs);
}
```

* 如果该决策信息是follow与overtake则，根据follow_s与overtake_s计算character_length,并将其放入boundary类中，如果是yield决策，则在原本的st_boundary加上yield_s 的buff
* 之后设置该boundary的type

```C++
STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {//decision是障碍物的纵向决策
    characteristic_length = std::fabs(decision.follow().distance_s());
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);//通过让行距离增加boundary
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->set_path_st_boundary(boundary);
```

```c++
STBoundary STBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}
```

## 将计算好的所有障碍物的st_boundary放入boundaries中

```C++
  std::vector<const STBoundary *> boundaries;
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }
```

## 计算道路的限速

```C++
  SpeedLimitDecider speed_limit_decider(speed_bounds_config_, reference_line,
                                        path_data);

  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

**首先根据地图限速信息得到初始限速**

**之后根据path的曲率获取限速，计算公式$ sqrt(flag/kappa)$**

**之后若是该障碍物决策为横向nudge,若是静态障碍物，则限速为道路限速的0.6，动态障碍物限速为道路限速的0.8**

```C++
Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();

  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();
    if (reference_line_s > reference_line_.Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // (1) speed limit from map
    double speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(reference_line_s);

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    //2.0/
    const double speed_limit_from_centripetal_acc =
        std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
                  std::fmax(std::fabs(discretized_path.at(i).kappa()),
                            speed_bounds_config_.minimal_kappa()));//0.00001

    // (3) speed limit from nudge obstacles
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    double speed_limit_from_nearby_obstacles =
        std::numeric_limits<double>::max();
    const double collision_safety_range =
        speed_bounds_config_.collision_safety_range();
    for (const auto* ptr_obstacle : obstacles.Items()) {
      if (ptr_obstacle->IsVirtual()) {
        continue;
      }
      if (!ptr_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */

      // TODO(all): potential problem here;
      // frenet and cartesian coordinates are mixed.
      const double vehicle_front_s =
          reference_line_s + vehicle_param_.front_edge_to_center();//车头s值
      const double vehicle_back_s =
          reference_line_s - vehicle_param_.back_edge_to_center();//车尾s值
      const double obstacle_front_s =
          ptr_obstacle->PerceptionSLBoundary().end_s();
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s();

      if (vehicle_front_s < obstacle_back_s ||
          vehicle_back_s > obstacle_front_s) {
        continue;
      }//车在障碍物后面或者车在障碍物前面

      const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path.at(i).l();

      // obstacle is on the right of ego vehicle (at path point i)
      bool is_close_on_left =
          (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center() -
               collision_safety_range <
           ptr_obstacle->PerceptionSLBoundary().end_l());

      // obstacle is on the left of ego vehicle (at path point i)
      bool is_close_on_right =
          (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
          (ptr_obstacle->PerceptionSLBoundary().start_l() -
               collision_safety_range <
           frenet_point_l + vehicle_param_.left_edge_to_center());

      // TODO(all): dynamic obstacles do not have nudge decision
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (ptr_obstacle->IsStatic()) {
          nudge_speed_ratio =
              speed_bounds_config_.static_obs_nudge_speed_ratio();//0.6
        } else {
          nudge_speed_ratio =
              speed_bounds_config_.dynamic_obs_nudge_speed_ratio();//0.8
        }
        speed_limit_from_nearby_obstacles =
            nudge_speed_ratio * speed_limit_from_reference_line;//地图限速
        break;
      }
    }//障碍物遍历结束

    double curr_speed_limit = 0.0;
    if (FLAGS_enable_nudge_slowdown) {//true
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc,
                              speed_limit_from_nearby_obstacles}));
    } else {
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc}));
    }
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }//离散点遍历结束
  return Status::OK();
}
```

## 之后将boundaries，speed_limit，巡航速度等信息放入st_graph_data中

```C++

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);//st_bound与vt_bound在st_bound_decider中输入

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);
```

