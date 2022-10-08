<center><span style="font-size:2rem;font-weight:bold;">ST_Bound_decider解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

该task的作用是：初始化ST坐标系，并将障碍物映射到ST上，提取相应的STbound，并对障碍物的boundary类型进行判定

流程如下：



# 函数处理

## 首先获得每个障碍物的st_boundry

```C++
  InitSTBoundsDecider(*frame, reference_line_info);
  //得到每个障碍物的st_boundary,
  //boundary = {point_pairs，true}
  //将lower_points与upper_points序号对应的放入boundary中
  //    point_pairs.emplace_back(
  //    STPoint(lower_points.at(i).s(), lower_points.at(i).t()),//
  //    STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  //讲台障碍物有4个点，动态障碍物有轨迹点*2个点（上下）
  //并构建obs_t_edges_向量
  //obs_t_edges_类型：
  //[0]=[true,起始时间，左下边界点的s,左上边界点的s,obs_id]
  //[1]=[false,终止时间，右下边界点的s,右上边界点的s,obs_id]
```

具体流程

* 首先根据path以及config初始化st_obstacles_processor_

```C++
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data,//7s
                               path_decision, injector_->history());//类内赋值
```

```C++
void STObstaclesProcessor::Init(const double planning_distance,
                                const double planning_time,
                                const PathData& path_data,
                                PathDecision* const path_decision,
                                History* const history) {
  planning_time_ = planning_time;//7.0
  planning_distance_ = planning_distance;//长度
  path_data_ = path_data;//最优的规划路径
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  adc_path_init_s_ = path_data_.discretized_path().front().s();//规划点
  path_decision_ = path_decision;
  history_ = history;

  obs_t_edges_.clear();
  obs_t_edges_idx_ = 0;

  obs_id_to_st_boundary_.clear();
  obs_id_to_decision_.clear();
  candidate_clear_zones_.clear();
  obs_id_to_alternative_st_boundary_.clear();
}
```

* 之后计算所有障碍物的st_boundary

```C++
st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);//将障碍物映射到ST坐标系
  //得到每个障碍物的st_boundary,并将不和path_data发生交集的障碍物设置横向纵向忽略决策
  //boundary = {point_pairs，true}
  //将lower_points与upper_points序号对应的放入boundary中
  //    point_pairs.emplace_back(
  //    STPoint(lower_points.at(i).s(), lower_points.at(i).t()),//
  //    STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  //静态障碍物有4个点，动态障碍物有轨迹点*2个点（上下）
  //并构建obs_t_edges_向量
  //obs_t_edges_类型（每个障碍物对应有两个向量，true,false）：
  //[0]=[true,起始时间，左下边界点的s,左上边界点的s,obs_id]
  //[1]=[false,终止时间，右下边界点的s,右上边界点的s,obs_id]
```

具体过程

1. 首先将path中处于两条lane中间的path段提取出来，用于计算动态障碍物的是否应该caution

```c++
  //将处于正在变道的点放入adc_low_road_right_segments_中
  //将处于两条lane交界处的点放入adc_low_road_right_segments_中(片段)
  bool is_adc_low_road_right_beginning = true;
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;//对path_pt_info解码分别赋值，
    if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
        path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      if (is_adc_low_road_right_beginning) {
        adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
        is_adc_low_road_right_beginning = false;
      } else {
        adc_low_road_right_segments_.back().second = path_pt_s;
      }
    } else if (path_pt_type == PathData::PathPointType::IN_LANE) {
      if (!is_adc_low_road_right_beginning) {
        is_adc_low_road_right_beginning = true;
      }
    }
  }
```

2. 之后遍历每个障碍物，分别计算障碍物的st_boundary

```C++
    //计算障碍物与path相交的点（相交的第一个点与最后一个点对应的s,与对应的时间）
    //将静态障碍物的第一个重叠点的s值与起始时间0.0，放入lower_points
    //将静态障碍物的第一个重叠点的s值与终止时间7.0，放入lower_points
    //将静态障碍物的最后一个重叠点的s值与起始时间0.0，放入upper_points
    //将静态障碍物的最后一个重叠点的s值与终止时间7.0，放入upper_points
    //遍历障碍物的轨迹点，将其每一个轨迹的第一个重叠点的s值与起始时间relative_time，放入lower_points
    //遍历障碍物的轨迹点，将其每一个轨迹的最后一个重叠点的s值与时间relative_time，放入upper_points
    //且如果该障碍物是静态障碍物，则obs_caution_end_t=7.0，若其实动态障碍物且第一个轨迹点处于正在变道中，则obs_caution_end_t=relative_time
    if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,
                                   &is_caution_obstacle, &obs_caution_end_t)) {
      // Obstacle doesn't appear on ST-Graph.
      continue;
    }
```

2.1 `ComputeObstacleSTBoundary`函数

**如果该障碍物不存在轨迹，则根据该障碍物的box计算与path重叠的s**

```C++
    if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,//0.1
                        &overlapping_s)) {
      lower_points->emplace_back(overlapping_s.first, 0.0);
      lower_points->emplace_back(overlapping_s.first, planning_time_);
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);
    }
```

`GetOverlappingS`函数

首先分别利用二分法计算与path重叠的轨迹点的idx

```C++
 int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);//找到障碍物与path重叠的第一个点（detail）
  ADEBUG << "The index before is " << pt_before_idx;
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);//找到障碍物与path重叠的最后一个点
  ADEBUG << "The index after is " << pt_after_idx;
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {//如果重叠的第一个点就是最后一个path点则没有重叠区域
    return false;
  }
  if (pt_after_idx == 0) {//重叠的最后一点是起始点，则false
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }
```

之后将范围缩小到pt_before_idx-pt_after_idx，利用两个box是否重叠计算重叠的s值

```C++
    ADEBUG << "At ADC path index = " << i << " :";
    //在每个重叠的点计算车的box，检查车的box与obs的box是否重叠
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {//adc_l_buffer==0.1
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();//重叠区域的第一个点overlapping_s
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }//处于obs中的path点遍历结束
  if (!has_overlapping) {
    return false;
  }
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->second = adc_path_points[i + 1].s();//重叠的最后一个点overlapping_s
      ADEBUG << "There is overlapping.";
      break;
    }
  }
```

**将计算到的overlap区域，放入**

```C++
lower_points->emplace_back(overlapping_s.first, 0.0);
lower_points->emplace_back(overlapping_s.first, planning_time_);
upper_points->emplace_back(overlapping_s.second, 0.0);
upper_points->emplace_back(overlapping_s.second, planning_time_);
```

**如果该障碍物存在轨迹，则遍历该障碍物的轨迹点，重叠区域的计算与上相同，但是不止存在两个**lower_points和upper_points，每个obs轨迹的**t**对应的OBS_BOX都会计算，并保存

```C++
    // Processing a dynamic obstacle.
    // Go through every occurrence of the obstacle at all timesteps, and
    // figure out the overlapping s-max and s-min one by one.
    bool is_obs_first_traj_pt = true;
    for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {//遍历障碍物的每个轨迹
      // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
      // disjoint segments (happens very rarely), we merge them into one.
      // In the future, this could be considered in greater details rather
      // than being approximated.
      const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);//得到轨迹点是obs的box
      ADEBUG << obs_box.DebugString();
      std::pair<double, double> overlapping_s;
      //overlapping_s保存的是该path与障碍相交的起始的点s,最后相交的点s
      if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,//0.1
                          &overlapping_s)) {
        ADEBUG << "Obstacle instance is overlapping with ADC path.";
        lower_points->emplace_back(overlapping_s.first,
                                   obs_traj_pt.relative_time());//将重叠的第一个点与obs到达该点时的时间放入
        upper_points->emplace_back(overlapping_s.second,
                                   obs_traj_pt.relative_time());//将重叠的最后的点与obs到达该点时的时间放入
        //
        if (is_obs_first_traj_pt) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {//如果与障碍物重叠的点处于变道中，则is_caution_obstacle==true
              //即将其看为静态障碍物
            *is_caution_obstacle = true;
          }
        }
        if ((*is_caution_obstacle)) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *obs_caution_end_t = obs_traj_pt.relative_time();//设置该障碍物需要的end时间为该动态障碍物的轨迹时间
          }
        }
      }//计算该障碍物点重叠点结束
      is_obs_first_traj_pt = false;
    }//障碍物轨迹点遍历结束
    if (lower_points->size() == 1) {
      lower_points->emplace_back(lower_points->front().s(),
                                 lower_points->front().t() + 0.1);
      upper_points->emplace_back(upper_points->front().s(),
                                 upper_points->front().t() + 0.1);
    }
  }
```

3. 将计算到的lower_points和upper_points，对应起来，并放入boundary中

```C++
    auto boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
```

```C++
STBoundary STBoundary::CreateInstanceAccurate(
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
  return STBoundary(point_pairs, true);
}
```

4. 如果静态障碍物处于KC区域中，将障碍物的boundary的设置为kc区域

```C++
  if (std::get<0>(closest_stop_obstacle) != "NULL") {//有静态障碍物距离path很近时为obs的id
    std::string closest_stop_obs_id;
    STBoundary closest_stop_obs_boundary;
    Obstacle* closest_stop_obs_ptr;
    std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
             closest_stop_obs_ptr) = closest_stop_obstacle;
    ADEBUG << "Closest obstacle ID = " << closest_stop_obs_id;
    // Go through all Keep-Clear zones, and see if there is an even closer
    // stop fence due to them.
    if (!closest_stop_obs_ptr->IsVirtual()) {
      for (const auto& clear_zone : candidate_clear_zones_) {//遍历keep_clear区域
        const auto& clear_zone_boundary = std::get<1>(clear_zone);
        if (closest_stop_obs_boundary.min_s() >= clear_zone_boundary.min_s() &&
            closest_stop_obs_boundary.min_s() <= clear_zone_boundary.max_s()) {//如果静态障碍物处于kc区
          std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
                   closest_stop_obs_ptr) = clear_zone;//将改静态障碍物的st_boundary设为kc的边界
          ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
          break;
        }
      }//kc区域遍历结束
    }
```

5. 将障碍物的boundary浓缩为两条边，并对其排序

```C++
  for (const auto& it : obs_id_to_st_boundary_) {//遍历所有map到st坐标的障碍物
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);//[true,起始时间，左下边界点的s,左上边界点的s,]
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);//[false,终止时间，右下边界点的s,右上边界点的s,]
  }
  // Sort the edges.
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),//排序
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return Status::OK();
}
```

* 之后根据期望速度：15m/s,初始化guide_line，用于生成st_bound,并设置最大加速度与减速度与当前速度信息，用来计算动力学限制

```C++
  if (path_data.is_optimized_towards_trajectory_reference()) {//在path的优化中设为true
    st_guide_line_.Init(desired_speed,
                        injector_->learning_based_data()
                            ->learning_data_adc_future_trajectory_points());//根据初始速度，加速度初始化st_guide_line_
  } else {
    st_guide_line_.Init(desired_speed);
  }
  static constexpr double max_acc = 2.5;
  static constexpr double max_dec = 5.0;
  static constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());//动力学约束信息赋值
```

```C++
void STGuideLine::Init(double desired_v) {
  s0_ = 0.0;
  t0_ = 0.0;
  v0_ = desired_v;
}
```

```C++
void STDrivingLimits::Init(const double max_acc, const double max_dec,
                           const double max_v, double curr_v) {
  max_acc_ = max_acc;
  max_dec_ = max_dec;
  max_v_ = max_v;
  upper_t0_ = 0.0;
  upper_v0_ = curr_v;
  upper_s0_ = 0.0;
  lower_t0_ = 0.0;
  lower_v0_ = curr_v;
  lower_s0_ = 0.0;
}
```

## 之后计算每个时间点t（分辨率为0.1）的DrivableBoundary，st_bound与vt_bound

```C++
  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  STBound regular_vt_bound;
  std::vector<std::pair<double, double>> st_guide_line;
  Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,
                                      &st_guide_line);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    reference_line_info->SetFallbackAcc(kSTFallbackAcceleration);
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    const std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);

  // Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_guide_line,
                     st_graph_debug);

  return Status::OK();
}
```

### GenerateRegularSTBound函数

* 首先遍历时间t（分辨率0.1）对st_boud与vt_bound进行初始化

```C++
  //初始化每个t时刻的最小s为无穷小，最大s为无穷大
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();//7.0
       curr_t += kSTBoundsDeciderResolution) {//0.1
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }
```

* 之后利用扫线法为每个t计算可行的st_bound与vt_bound

  * 首先根据运动学公式计算粗略的bound

  ```C++
    //以最大的加速度与减速度形式t时刻后的s值
      auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);//得到车的动力学限制
      //driving_limits_bound.first：从当前速度以最大减速度行驶到当前时刻t时行驶的距离
      //driving_limits_bound.second：从当前速度以最大加速度行驶到当前时刻t时行驶的距离
      s_lower = std::fmax(s_lower, driving_limits_bound.first);
      s_upper = std::fmin(s_upper, driving_limits_bound.second);
      ADEBUG << "Bounds for s due to driving limits are "
             << "s_upper = " << s_upper << ", s_lower = " << s_lower;
  ```

  * 之后根据障碍物信息计算bound

  ```C++
      //available_s_bounds是此时刻有效的s的范围（可能存在多个障碍物，因此为vector）
      //available_obs_decisions每个gap的决策
      std::vector<std::pair<double, double>> available_s_bounds;
      std::vector<ObsDecSet> available_obs_decisions;
      if (!st_obstacles_processor_.GetSBoundsFromDecisions(//重点
              t, &available_s_bounds, &available_obs_decisions)) {
        const std::string msg =
            "Failed to find a proper boundary due to obstacles.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
  ```

  GetSBoundsFromDecisions函数

  **首先筛选出egde在t之前且使lower的boundary边**

  ```C++
    while (obs_t_edges_idx_ < static_cast<int>(obs_t_edges_.size()) &&
           std::get<1>(obs_t_edges_[obs_t_edges_idx_]) <= t) {
      if (std::get<0>(obs_t_edges_[obs_t_edges_idx_]) == 0 &&//==0false表示upper点
          std::get<1>(obs_t_edges_[obs_t_edges_idx_]) == t) {//时间大于等于t表示该obs无影响了（）
        break;
      }
      ADEBUG << "Seeing a new t-edge at t = "
             << std::get<1>(obs_t_edges_[obs_t_edges_idx_]);
      new_t_edges.push_back(obs_t_edges_[obs_t_edges_idx_]);
      ++obs_t_edges_idx_;
    }//相关的st_boundaries放入new_t_edges中
  ```

  **之后遍历筛选之后的boundary，并为其作出决策**

  ```C++
    //根据新的st_boundary,做出决策
    std::vector<ObsTEdge> ambiguous_t_edges;
    for (auto obs_t_edge : new_t_edges) {
      ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
             << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
             << std::get<3>(obs_t_edge) << "]";
      if (std::get<0>(obs_t_edge) == 1) {//lower
        if (std::get<2>(obs_t_edge) >= s_max) {//下边界大于s_max,该障碍物此时在车前，应该让行
          ADEBUG << "  Apparently, it should be yielded.";//
          obs_id_to_decision_[std::get<4>(obs_t_edge)] =//根据障碍物bound与s_max的关系做出决策
              DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                        std::get<3>(obs_t_edge), s_max);
          obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
              STBoundary::BoundaryType::YIELD);
        } else if (std::get<3>(obs_t_edge) <= s_min) {//车此时刻应该在障碍物前，因此需要超车
          ADEBUG << "  Apparently, it should be overtaken.";
          obs_id_to_decision_[std::get<4>(obs_t_edge)] =
              DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                        std::get<3>(obs_t_edge), s_min);
          obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
              STBoundary::BoundaryType::OVERTAKE);
        } else {
          ADEBUG << "  It should be further analyzed.";
          ambiguous_t_edges.push_back(obs_t_edge);//其他情况
        }
      }
    }
    // For ambiguous ones, enumerate all decisions and corresponding bounds.
    auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
    if (s_gaps.empty()) {
      return false;
    }
    //根据gap中间值与障碍物的boun的关系设定决策
    for (auto s_gap : s_gaps) {
      available_s_bounds->push_back(s_gap);
      std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
      for (auto obs_t_edge : ambiguous_t_edges) {
        std::string obs_id = std::get<4>(obs_t_edge);
        double obs_s_min = std::get<2>(obs_t_edge);
        double obs_s_max = std::get<3>(obs_t_edge);
        obs_decisions.emplace_back(
            obs_id,
            DetermineObstacleDecision(obs_s_min, obs_s_max,
                                      (s_gap.first + s_gap.second) / 2.0));
      }
      available_obs_decisions->push_back(obs_decisions);
    }
  
  ```

  * 之后在这些gap中找到最好的一个，更新st_bound与vt_boundd

## 最后将st_bound与vt_bound放入StGraphData

```C++
StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);
```

