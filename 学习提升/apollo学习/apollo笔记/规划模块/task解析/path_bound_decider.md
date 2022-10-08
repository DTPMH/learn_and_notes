<center><span style="font-size:2rem;font-weight:bold;">path_bound_decider</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总体概览

* 计算初始规划点的`adc_s`,`adc_l`值,并计算初始规划点处的`lane_width`
  ```C
  InitPathBoundsDecider(*frame, *reference_line_info);
  ```

  `InitPathBoundsDecider`函数具体[如下所示](# `InitPathBoundsDecider`函数)

* 之后首先计算`fallback`的`path_bound`,将`fallback`的`bound`的初始s值，以及s的分辨率(0.5)与`fallback_path_bound_pair`的`bound`放入`candidate_path_boundaries`中，并将其`label`设为`fallback`。
  ```c++
    PathBound fallback_path_bound;// 数据类型vector<>(s, l_min, l_max)
    //得到应急车道边界信息（需要根据规划点以及车速进行计算）->fallback_path_bound
    //根据lane的边界信息，以及车宽以及buff计算应急车道
    Status ret =
        GenerateFallbackPathBound(*reference_line_info, &fallback_path_bound);//
    if (!ret.ok()) {
      ADEBUG << "Cannot generate a fallback path bound.";
      return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
    }
    if (fallback_path_bound.empty()) {
      const std::string msg = "Failed to get a valid fallback path boundary";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    if (!fallback_path_bound.empty()) {//检查规划点的l值是否处于应急路径边界中
      CHECK_LE(adc_frenet_l_, std::get<2>(fallback_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(fallback_path_bound[0]));
    }
    // Update the fallback path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> fallback_path_bound_pair;
    //将应急path的bound放入引导线的fallback_path_bound_pair中
    for (size_t i = 0; i < fallback_path_bound.size(); ++i) {//遍历生成的边界
      fallback_path_bound_pair.emplace_back(std::get<1>(fallback_path_bound[i]),
                                            std::get<2>(fallback_path_bound[i]));
    }
  
    candidate_path_boundaries.emplace_back(std::get<0>(fallback_path_bound[0]),
                                           kPathBoundsDeciderResolution,
                                           fallback_path_bound_pair);
    candidate_path_boundaries.back().set_label("fallback");
  ```

  **`GenerateFallbackPathBound`函数详细[如下](#`GenerateFallbackPathBound`函数)**

* 如果此时处于pull_over场景中，则计算pull_over的path_bound，将`pull_over`的`bound`的初始s值，以及s的分辨率(0.5)与`pull_over_path_bound_pair`的`bound`放入`candidate_path_boundaries`中，并将其`label`设为`regular/pullover`,并返回。

```C++
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  const bool plan_pull_over_path = pull_over_status->plan_pull_over_path();//默认为false,pull_over场景下为true
  if (plan_pull_over_path) {
    PathBound pull_over_path_bound;
    //通过计算road的边界形成
    Status ret = GeneratePullOverPathBound(*frame, *reference_line_info,
                                           &pull_over_path_bound);
    if (!ret.ok()) {
      AWARN << "Cannot generate a pullover path bound, do regular planning.";
    } else {
      ACHECK(!pull_over_path_bound.empty());
      CHECK_LE(adc_frenet_l_, std::get<2>(pull_over_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(pull_over_path_bound[0]));

      // Update the fallback path boundary into the reference_line_info.
      std::vector<std::pair<double, double>> pull_over_path_bound_pair;
      for (size_t i = 0; i < pull_over_path_bound.size(); ++i) {
        pull_over_path_bound_pair.emplace_back(
            std::get<1>(pull_over_path_bound[i]),
            std::get<2>(pull_over_path_bound[i]));
      }
      candidate_path_boundaries.emplace_back(
          std::get<0>(pull_over_path_bound[0]), kPathBoundsDeciderResolution,
          pull_over_path_bound_pair);
      candidate_path_boundaries.back().set_label("regular/pullover");
  //将pull-over(靠边停车)的bound的初始s值，以及s的分辨率与pull_over_path_bound的bound放入candidate_path_boundaries中
  //并将其label设为fallback
      reference_line_info->SetCandidatePathBoundaries(
          std::move(candidate_path_boundaries));
      ADEBUG << "Completed pullover and fallback path boundaries generation.";

      // set debug info in planning_data
      auto* pull_over_debug = reference_line_info->mutable_debug()
                                  ->mutable_planning_data()
                                  ->mutable_pull_over();
      pull_over_debug->mutable_position()->CopyFrom(
          pull_over_status->position());
      pull_over_debug->set_theta(pull_over_status->theta());
      pull_over_debug->set_length_front(pull_over_status->length_front());
      pull_over_debug->set_length_back(pull_over_status->length_back());
      pull_over_debug->set_width_left(pull_over_status->width_left());
      pull_over_debug->set_width_right(pull_over_status->width_right());

      return Status::OK();
    }
  }
```

**`GeneratePullOverPathBound`函数详细[如下](#`GeneratePullOverPathBound`函数)**

* 若是不处于pull_over场景，则查看是否处于lane_change场景，若是处于lane_change场景，则生成lane_change的path_boundary，并返回label:`regular/lanechange`

```c++
  if (FLAGS_enable_smarter_lane_change &&//true
      reference_line_info->IsChangeLanePath()) {//该引导线是lane_change的引导线
    PathBound lanechange_path_bound;
    Status ret = GenerateLaneChangePathBound(*reference_line_info,
                                             &lanechange_path_bound);//与正常车道计算一样
    if (!ret.ok()) {
      ADEBUG << "Cannot generate a lane-change path bound.";
      return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
    }
    if (lanechange_path_bound.empty()) {
      const std::string msg = "Failed to get a valid fallback path boundary";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {//false
      CHECK_LE(adc_frenet_l_, std::get<2>(lanechange_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(lanechange_path_bound[0]));
    }
    // Update the fallback path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> lanechange_path_bound_pair;
    for (size_t i = 0; i < lanechange_path_bound.size(); ++i) {
      lanechange_path_bound_pair.emplace_back(
          std::get<1>(lanechange_path_bound[i]),
          std::get<2>(lanechange_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(
        std::get<0>(lanechange_path_bound[0]), kPathBoundsDeciderResolution,
        lanechange_path_bound_pair);
    candidate_path_boundaries.back().set_label("regular/lanechange");
    RecordDebugInfo(lanechange_path_bound, "", reference_line_info);
    reference_line_info->SetCandidatePathBoundaries(
        std::move(candidate_path_boundaries));
    ADEBUG << "Completed lanechange and fallback path boundaries generation.";
    return Status::OK();
  }

```

**`GenerateLaneChangePathBound`函数详细[如下](#`GenerateLaneChangePathBound`函数)**

* 若是以上均不存在，则生成regularpath_boundary，若是需要借道的话，则生成不借道，借道的path_boundary均会生成，label为：

```C++
"regular/self"
"regular/left/forward(reverse)"
"regular/right/forward(reverse)"
```

* 首先根据借道信息，将相应借道决策放入lane_borrow_info_list中

```C++
  if (reference_line_info->is_path_lane_borrow()) {//如果该参考线需要借道
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::LEFT_BORROW);
      } else if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::RIGHT_BORROW);
      }
    }
  }
```

* 之后对不同的借道决策生成不同的借道boundary

```C++
  for (const auto& lane_borrow_info : lane_borrow_info_list) {
    PathBound regular_path_bound;
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    Status ret = GenerateRegularPathBound(
        *reference_line_info, lane_borrow_info, &regular_path_bound,
        &blocking_obstacle_id, &borrow_lane_type);
    if (!ret.ok()) {
      continue;
    }
    if (regular_path_bound.empty()) {
      continue;
    }
    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, std::get<2>(regular_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(regular_path_bound[0]));
    }

    // Update the path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> regular_path_bound_pair;
    for (size_t i = 0; i < regular_path_bound.size(); ++i) {
      regular_path_bound_pair.emplace_back(std::get<1>(regular_path_bound[i]),
                                           std::get<2>(regular_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(std::get<0>(regular_path_bound[0]),
                                           kPathBoundsDeciderResolution,
                                           regular_path_bound_pair);
    std::string path_label = "";
    switch (lane_borrow_info) {
      case LaneBorrowInfo::LEFT_BORROW:
        path_label = "left";
        break;
      case LaneBorrowInfo::RIGHT_BORROW:
        path_label = "right";
        break;
      default:
        path_label = "self";
        // exist_self_path_bound = true;
        // regular_self_path_bound = regular_path_bound;
        break;
    }
    // RecordDebugInfo(regular_path_bound, "", reference_line_info);
    candidate_path_boundaries.back().set_label(
        absl::StrCat("regular/", path_label, "/", borrow_lane_type));
    candidate_path_boundaries.back().set_blocking_obstacle_id(
        blocking_obstacle_id);
  }
```

**`GenerateRegularPathBound`函数详细[如下](#`GenerateRegularPathBound`函数)**

# 各个分函数详细介绍

## `InitPathBoundsDecider`函数

* 首先将起始点的笛卡尔坐标系转换为frent坐标系

```C++
auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first[0];
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
```

* 之后计算起始点到达lane中线的距离，记为`adc_l_to_lane_center_`

```C++
  double offset_to_map = 0.0;
  reference_line.GetOffsetToMap(adc_frenet_s_, &offset_to_map);
  adc_l_to_lane_center_ = adc_frenet_l_ + offset_to_map;
```

* 计算起始点处的lane宽

```C++
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }
```

## `GenerateFallbackPathBound`函数

* 首先根据初始规划点处的s值，以及前向距离，并以`0.5`作为分辨率，向前延伸，计算每个s点处的path_bound。
  前向距离的计算：
  $$
  forward = min(reference\_length,max(100,v*8))
  $$
  将每个`s`处的path_bound初始化为`[s,(-OO,+OO)]`

  代码：
  ```C++
    if (!InitPathBoundary(reference_line_info, path_bound)) {
      const std::string msg = "Failed to initialize fallback path boundaries.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  ```

```C++
bool PathBoundsDecider::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}
```

* 之后根据adc的位置与lane开始计算fallback的path_bound
  
  ```C++
    if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                    LaneBorrowInfo::NO_BORROW, 0.5, path_bound,
                                    &dummy_borrow_lane_type, true)) {
      const std::string msg =
          "Failed to decide a rough fallback boundary based on "
          "road information.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  ```
  
  * 首先遍历每个s点
  
    * 根据该s点的值，计算当前lane在该点处的左右宽度：
  
    ```C++
    reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                         &curr_lane_right_width)
    ```
  
    * 计算瞬时横向位移
  
    ```C++
    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                  adc_frenet_ld_ * adc_frenet_ld_ /
                                  kMaxLateralAccelerations / 2.0;//横向的瞬时位移
    ```
  
    * 将该点处的lane的左边宽度赋正值，右边宽度赋负值
  
    ```C++
    double curr_left_bound_lane =
            curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                        ? curr_neighbor_lane_width
                                        : 0.0);//借道左边的宽度
    
        double curr_right_bound_lane =
            -curr_lane_right_width -
            (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                 ? curr_neighbor_lane_width
                 : 0.0);
    ```
  
    * 根据adc目前的位置计算对应adc的左右边界
  
    左边边界的值等于adc与lane中心线的距离+瞬时横向位移+0.5的车宽+adc_buff(0.5)，右边边界的值等于adc与lane中心线的距离-0.5的车宽-adc_buff(0.5)
  
    ```C++
      double curr_left_bound_adc =
              std::fmax(adc_l_to_lane_center_,
                        adc_l_to_lane_center_ + ADC_speed_buffer) +
              GetBufferBetweenADCCenterAndEdge() + ADC_buffer;//GetBufferBetweenADCCenterAndEdge=车的一半宽，ADC_buffer=0.5
    
          double curr_right_bound_adc =
              std::fmin(adc_l_to_lane_center_,
                        adc_l_to_lane_center_ + ADC_speed_buffer) -
              GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
    ```
  
    * 最终取adc边界与lane边界相应的最大最小值为该点处的bound
  
    ```C++
     curr_left_bound =
              std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;//根据车的范围，选择最大的bound
    
      curr_right_bound =
              std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
              offset_to_map;//最右边的边界
    ```
  
    * 根据buff值更新该s点处的path_bound，即两边剪去0.5的车宽
  
    ```C++
      double new_l_min =
          std::fmax(std::get<1>((*path_boundaries)[idx]),
                    right_bound + right_adc_buffer_coeff *
                                      GetBufferBetweenADCCenterAndEdge());//GetBufferBetweenADCCenterAndEdge返回的是0.5车宽
      // Update the left bound (l_max):
      double new_l_max = std::fmin(
          std::get<2>((*path_boundaries)[idx]),
          left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());
    
      // Check if ADC is blocked.
      // If blocked, don't update anything, return false.
      if (new_l_min > new_l_max) {
        ADEBUG << "Path is blocked at idx = " << idx;
        return false;
      }
      // Otherwise, update path_boundaries and center_line; then return true.
      std::get<1>((*path_boundaries)[idx]) = new_l_min;//更新该boundary
      std::get<2>((*path_boundaries)[idx]) = new_l_max;
    ```
  
  * 根据上一步阻挡的idx，对path_bound进行修剪

## `GeneratePullOverPathBound`函数

* 首先同上，先初始化path_boundary

```C++
  //[s,{-OO-+OO}]
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

* 之后根据road的边界计算粗略的path_boundary

```C++
  if (!GetBoundaryFromRoads(reference_line_info, path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

`GetBoundaryFromRoads`函数

1. 首先遍历每个s点

   1. 根据该s点的值计算此时road的边界

   ```C++
   reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                        &curr_road_right_width)
   ```

   2. 将左边界赋为正值，右边界赋为负值

   ```C++
     double curr_left_bound = curr_road_left_width;
       double curr_right_bound = -curr_road_right_width;
   ```

   3. 将该s点的值根据buff（即半个车宽）更新，将两边收缩半个车宽

   ```C++
       if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                         path_bound)) {
         path_blocked_idx = static_cast<int>(i);
       }
       if (path_blocked_idx != -1) {
         break;
       }
   ```

2. 根据上步的idx，修剪path_bound

* 之后将边界值从道路中心转移到引导线上

```C++
  ConvertBoundarySAxisFromLaneCenterToRefLine(reference_line_info, path_bound);
```

```C++
void PathBoundsDecider::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    std::get<1>((*path_bound)[i]) -= refline_offset_to_lane_center;
    std::get<2>((*path_bound)[i]) -= refline_offset_to_lane_center;
  }
}
```

* 之后根据当前lane的宽度，更新左边界，因为默认靠右边停车，因此将左边界设置为lane的左边界。

```C++
  //根据lane的bound更新path_bound(加上adc_l更新左边边界)
  UpdatePullOverBoundaryByLaneBoundary(reference_line_info, path_bound);
```

```C++
void PathBoundsDecider::UpdatePullOverBoundaryByLaneBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const auto pull_over_type = pull_over_status.pull_over_type();
  if (pull_over_type != PullOverStatus::PULL_OVER &&
      pull_over_type != PullOverStatus::EMERGENCY_PULL_OVER) {
    return;
  }

  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double curr_s = std::get<0>((*path_bound)[i]);
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (pull_over_type == PullOverStatus::PULL_OVER) {
      std::get<2>((*path_bound)[i]) = left_bound;
    } else if (pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER) {
      // TODO(all): use left/right lane boundary accordingly
      std::get<2>((*path_bound)[i]) = left_bound;
    }
  }
}
```

* 根据静态障碍物的位置调整path_bound

```C++
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

由于根据静态障碍物对path_boundary进行调整在其他边界生成时也用到了，因此将其单独说明，函数介绍见[跳转](# GetBoundaryFromStaticObstacles函数)

* 之后检测是否存在停车点，若是存在停车点，则得到该点的idx，

```C++
  if (pull_over_status->has_position()) {
    curr_idx = IsPointWithinPathBound(
        reference_line_info, pull_over_status->position().x(),
        pull_over_status->position().y(), *path_bound);
  }
```

* 若是不存在停车点，则搜索停车点

```C++
    if (!SearchPullOverPosition(frame, reference_line_info, *path_bound,
                                &pull_over_configuration)) {
      const std::string msg = "Failed to find a proper pull-over position.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
```

`SearchPullOverPosition`函数

1. **首先计算终点的s值**

```C++
FindDestinationPullOverS(frame, reference_line_info, path_bound,//得到目标点的s值
                                  &pull_over_s)
```

终点s值的计算

1. 先计算导航终点在引导线上的投影s

```C++
reference_line.XYToSL(routing_end.pose(), &destination_sl);//投影值
```

2. 如果终点与车头距离小于25m，则说明距离太近不能找到停车点，返回false

```C++
  if (destination_s - adc_end_s < config_.path_bounds_decider_config()
                                      .pull_over_destination_to_adc_buffer()) {//25
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }
```

2. **之后找到小于终点s值的第一个点的path_bound对应的idx**

```C++
  if (search_backward) {//找到小于pull_over_s的第一个点
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && std::get<0>(path_bound[idx]) > pull_over_s) {
      --idx;
    }
  }
```

3. **找到小于上一个idx1.5车长-5的idx，作为end_idx，计算停车点的s**

```C++
  // Search for a feasible location for pull-over.
  const double pull_over_space_length =//1.5*车长-5
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      FLAGS_obstacle_lon_start_buffer - FLAGS_obstacle_lon_end_buffer;//3-2
  const double pull_over_space_width =//1.25==>0.25*车宽
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =//一半车宽
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          std::get<0>(path_bound[idx]) - std::get<0>(path_bound.front()) >
              pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          std::get<0>(path_bound.back()) - std::get<0>(path_bound[idx]) >
              pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    //停车点不能在十字路口
    double pt_ref_line_s = std::get<0>(path_bound[idx]);
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info.reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::BaseMap().GetJunctions(hdmap_point, 1.0, &junctions);
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    while ((search_backward && j >= 0 &&//j是目标点在path_boundary中的idx
            std::get<0>(path_bound[idx]) - std::get<0>(path_bound[j]) <
                pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            std::get<0>(path_bound[j]) - std::get<0>(path_bound[idx]) <
                pull_over_space_length)) {
      double curr_s = std::get<0>(path_bound[j]);
      double curr_right_bound = std::fabs(std::get<1>(path_bound[j]));
      double curr_road_left_width = 0;
      double curr_road_right_width = 0;
      reference_line_info.reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      ADEBUG << "s[" << curr_s << "] curr_road_left_width["
             << curr_road_left_width << "] curr_road_right_width["
             << curr_road_right_width << "]";
      if (curr_road_right_width - (curr_right_bound + adc_half_width) >
          config_.path_bounds_decider_config().pull_over_road_edge_buffer()) {//0.15（停车时须靠路边0.15米）
        AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }
      const double right_bound = std::get<1>(path_bound[j]);
      const double left_bound = std::get<2>(path_bound[j]);
      ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
             << "]";
      if (left_bound - right_bound < pull_over_space_width) {
        AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }

      j = search_backward ? j - 1 : j + 1;
    }//while结束
    if (j < 0) {
      return false;
    }
    if (is_feasible_window) {
      has_a_feasible_window = true;
      const auto& reference_line = reference_line_info.reference_line();
      // estimate pull over point to have the vehicle keep same safety distance
      // to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +//1.5
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;//1.5

      int start_idx = j;
      int end_idx = idx;
      if (!search_backward) {
        start_idx = idx;
        end_idx = j;
      }
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));
```

4. **计算停车点的横向L值,右边界+0.125车宽+0.05**

```C++
    const double pull_over_l =
          std::get<1>(pull_over_point) + pull_over_space_width / 2.0 + FLAGS_pull_over_offset;//0.05，l的范围是boundary往里0.125的车宽
```

* 之后修剪停车点+20个点之后的path_boundary

```C++
  //只留pullover终点后20个boundary
  while (static_cast<int>(path_bound->size()) - 1 >
         curr_idx + kNumExtraTailBoundPoint) {//20
    path_bound->pop_back();
  }
  for (size_t idx = curr_idx + 1; idx < path_bound->size(); ++idx) {
    std::get<1>((*path_bound)[idx]) = std::get<1>((*path_bound)[curr_idx]);
    std::get<2>((*path_bound)[idx]) = std::get<2>((*path_bound)[curr_idx]);
  }
```

## `GenerateLaneChangePathBound`函数

* 首先同理，初始化path_boundary

```C++
  //根据车目前的位置，根据设定的delta s初始化boundary数组[0,min-max]
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

* 之后根据lane与adc的位置计算粗略的path_bounday，与fallback计算一样，只有adc_buff不一致

  ```C++
    if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                    LaneBorrowInfo::NO_BORROW, 0.1, path_bound,
                                    &dummy_borrow_lane_type, true)) {
      const std::string msg =
          "Failed to decide a rough boundary based on "
          "road information.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  ```

  `GetBoundaryFromLanesAndADC`函数

  * 首先遍历每个s点

    * 根据该s点的值，计算当前lane在该点处的左右宽度：

    ```C++
    reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                         &curr_lane_right_width)
    ```

    * 计算瞬时横向位移

    ```C++
    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                  adc_frenet_ld_ * adc_frenet_ld_ /
                                  kMaxLateralAccelerations / 2.0;//横向的瞬时位移
    ```

    * 将该点处的lane的左边宽度赋正值，右边宽度赋负值

    ```C++
    double curr_left_bound_lane =
            curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                        ? curr_neighbor_lane_width
                                        : 0.0);//借道左边的宽度
    
        double curr_right_bound_lane =
            -curr_lane_right_width -
            (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                 ? curr_neighbor_lane_width
                 : 0.0);
    ```

    * 根据adc目前的位置计算对应adc的左右边界

    左边边界的值等于adc与lane中心线的距离+瞬时横向位移+0.5的车宽+adc_buff(0.1)，右边边界的值等于adc与lane中心线的距离-0.5的车宽-adc_buff(0.1)

    ```C++
      double curr_left_bound_adc =
              std::fmax(adc_l_to_lane_center_,
                        adc_l_to_lane_center_ + ADC_speed_buffer) +
              GetBufferBetweenADCCenterAndEdge() + ADC_buffer;//GetBufferBetweenADCCenterAndEdge=车的一半宽，ADC_buffer=0.5
    
          double curr_right_bound_adc =
              std::fmin(adc_l_to_lane_center_,
                        adc_l_to_lane_center_ + ADC_speed_buffer) -
              GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
    ```

    * 最终取adc边界与lane边界相应的最大最小值为该点处的bound

    ```C++
     curr_left_bound =
              std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;//根据车的范围，选择最大的bound
    
      curr_right_bound =
              std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
              offset_to_map;//最右边的边界
    ```

    * 根据buff值更新该s点处的path_bound，即两边剪去0.5的车宽

    ```C++
      double new_l_min =
          std::fmax(std::get<1>((*path_boundaries)[idx]),
                    right_bound + right_adc_buffer_coeff *
                                      GetBufferBetweenADCCenterAndEdge());//GetBufferBetweenADCCenterAndEdge返回的是0.5车宽
      // Update the left bound (l_max):
      double new_l_max = std::fmin(
          std::get<2>((*path_boundaries)[idx]),
          left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());
    
      // Check if ADC is blocked.
      // If blocked, don't update anything, return false.
      if (new_l_min > new_l_max) {
        ADEBUG << "Path is blocked at idx = " << idx;
        return false;
      }
      // Otherwise, update path_boundaries and center_line; then return true.
      std::get<1>((*path_boundaries)[idx]) = new_l_min;//更新该boundary
      std::get<2>((*path_boundaries)[idx]) = new_l_max;
    ```

  * 根据上一步阻挡的idx，对path_bound进行修剪

* 根据是否变道是否安全，修剪lane_change边界
  ```C++
    GetBoundaryFromLaneChangeForbiddenZone(reference_line_info, path_bound);
  ```

  首先判断变道是否安全，判断依据：

  * 目标车道存在动态障碍物，且该障碍物若是与adc同向的话，两者之间的距离差需要大于10m,若是方向则距离需要大于50m

  ```C++
  void PathBoundsDecider::GetBoundaryFromLaneChangeForbiddenZone(
      const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
    // Sanity checks.
    CHECK_NOTNULL(path_bound);
    const ReferenceLine& reference_line = reference_line_info.reference_line();
  
    // If there is a pre-determined lane-change starting position, then use it;
    // otherwise, decide one.
    auto* lane_change_status = injector_->planning_context()
                                   ->mutable_planning_status()
                                   ->mutable_change_lane();
    if (lane_change_status->is_clear_to_change_lane()) {
      ADEBUG << "Current position is clear to change lane. No need prep s.";
      lane_change_status->set_exist_lane_change_start_position(false);
      return;
    }
    double lane_change_start_s = 0.0;
    if (lane_change_status->exist_lane_change_start_position()) {
      common::SLPoint point_sl;
      reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                            &point_sl);
      lane_change_start_s = point_sl.s();
    } else {
      // TODO(jiacheng): train ML model to learn this.
      lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;//80
  
      // Update the decided lane_change_start_s into planning-context.
      common::SLPoint lane_change_start_sl;
      lane_change_start_sl.set_s(lane_change_start_s);
      lane_change_start_sl.set_l(0.0);
      common::math::Vec2d lane_change_start_xy;
      reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
      lane_change_status->set_exist_lane_change_start_position(true);
      lane_change_status->mutable_lane_change_start_position()->set_x(
          lane_change_start_xy.x());
      lane_change_status->mutable_lane_change_start_position()->set_y(
          lane_change_start_xy.y());
    }
  
    // Remove the target lane out of the path-boundary, up to the decided S.
    if (lane_change_start_s < adc_frenet_s_) {
      // If already passed the decided S, then return.
      // lane_change_status->set_exist_lane_change_start_position(false);
      return;
    }
    //修改该参考线上变道起始点前的path_boundary
    for (size_t i = 0; i < path_bound->size(); ++i) {
      double curr_s = std::get<0>((*path_bound)[i]);
      if (curr_s > lane_change_start_s) {
        break;
      }
      double curr_lane_left_width = 0.0;
      double curr_lane_right_width = 0.0;
      double offset_to_map = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_map);
      if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                      &curr_lane_right_width)) {
        double offset_to_lane_center = 0.0;
        reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
        curr_lane_left_width += offset_to_lane_center;
        curr_lane_right_width -= offset_to_lane_center;
      }
      curr_lane_left_width -= offset_to_map;
      curr_lane_right_width += offset_to_map;
  //如果汽车超出该lane的左边界，则将lane的左边界加上车宽作为新的右边界
      std::get<1>((*path_bound)[i]) =
          adc_frenet_l_ > curr_lane_left_width
              ? curr_lane_left_width + GetBufferBetweenADCCenterAndEdge()
              : std::get<1>((*path_bound)[i]);
      std::get<1>((*path_bound)[i]) =
          std::fmin(std::get<1>((*path_bound)[i]), adc_frenet_l_ - 0.1);
          //左边同理
      std::get<2>((*path_bound)[i]) =
          adc_frenet_l_ < -curr_lane_right_width
              ? -curr_lane_right_width - GetBufferBetweenADCCenterAndEdge()
              : std::get<2>((*path_bound)[i]);
      std::get<2>((*path_bound)[i]) =
          std::fmax(std::get<2>((*path_bound)[i]), adc_frenet_l_ + 0.1);
    }
  }
  ```

* 最后根据障碍物修剪path_boundary

```C++
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

## `GenerateRegularPathBound`函数

* 同样首先初始化path_boundary

```C++
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

* 根据lane的位置与adc的位置计算boundary

```c++
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_borrow_info, 0.1,
                                  path_bound, borrow_lane_type)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

`GetBoundaryFromLanesAndADC`函数：

1. 首先遍历每个s点

   * 根据该s点的值，计算当前lane在该点处的左右宽度：

   ```C++
   reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                        &curr_lane_right_width)
   ```

   * 根据借道的决策信息，获得邻居车道的信息，若是不借道，则直接返回false，若是借道，则查看该邻居lane是否为实线，若是则返回false,反之返回true

   ```C++
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {//检测能够借道(no_borrow时为false)
         hdmap::Id neighbor_lane_id;
         //左边借道
         if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
           // Borrowing left neighbor lane.
           if (reference_line_info.GetNeighborLaneInfo(
                   ReferenceLineInfo::LaneType::LeftForward, curr_s,
                   &neighbor_lane_id, &curr_neighbor_lane_width)) {//返回左边邻居的第一条道
             ADEBUG << "Borrow left forward neighbor lane.";
           } else if (reference_line_info.GetNeighborLaneInfo(
                          ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                          &neighbor_lane_id, &curr_neighbor_lane_width)) {
             borrowing_reverse_lane = true;
             ADEBUG << "Borrow left reverse neighbor lane.";
           } else {
             ADEBUG << "There is no left neighbor lane.";
           }
         }
         //右边借道 
         else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
           // Borrowing right neighbor lane.
           if (reference_line_info.GetNeighborLaneInfo(
                   ReferenceLineInfo::LaneType::RightForward, curr_s,
                   &neighbor_lane_id, &curr_neighbor_lane_width)) {
             ADEBUG << "Borrow right forward neighbor lane.";
           } else if (reference_line_info.GetNeighborLaneInfo(
                          ReferenceLineInfo::LaneType::RightReverse, curr_s,
                          &neighbor_lane_id, &curr_neighbor_lane_width)) {
             borrowing_reverse_lane = true;
             ADEBUG << "Borrow right reverse neighbor lane.";
           } else {
             ADEBUG << "There is no right neighbor lane.";
           }
         }
   ```

   * 计算瞬时横向位移

   ```C++
   double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                 adc_frenet_ld_ * adc_frenet_ld_ /
                                 kMaxLateralAccelerations / 2.0;//横向的瞬时位移
   ```

   * 将该点处的lane的左边宽度赋正值，右边宽度赋负值

   ```C++
   double curr_left_bound_lane =
           curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                       ? curr_neighbor_lane_width
                                       : 0.0);//借道左边的宽度
   
       double curr_right_bound_lane =
           -curr_lane_right_width -
           (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                ? curr_neighbor_lane_width
                : 0.0);
   ```

   * 根据buff值更新该s点处的path_bound，即两边剪去0.5的车宽

   ```C++
     double new_l_min =
         std::fmax(std::get<1>((*path_boundaries)[idx]),
                   right_bound + right_adc_buffer_coeff *
                                     GetBufferBetweenADCCenterAndEdge());//GetBufferBetweenADCCenterAndEdge返回的是0.5车宽
     // Update the left bound (l_max):
     double new_l_max = std::fmin(
         std::get<2>((*path_boundaries)[idx]),
         left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());
   
     // Check if ADC is blocked.
     // If blocked, don't update anything, return false.
     if (new_l_min > new_l_max) {
       ADEBUG << "Path is blocked at idx = " << idx;
       return false;
     }
     // Otherwise, update path_boundaries and center_line; then return true.
     std::get<1>((*path_boundaries)[idx]) = new_l_min;//更新该boundary
     std::get<2>((*path_boundaries)[idx]) = new_l_max;
   ```

2. 根据上一步阻挡的idx，对path_bound进行修剪

* 之后根据障碍物信息调整boundary信息

```C++
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```



# `GetBoundaryFromStaticObstacles`函数

GetBoundaryFromStaticObstacles函数在path_bound_decider中是重要的一环，决定着path是否会与静态障碍物相撞，过程如下

* 首先对障碍物进行扫线法排序

```C++
  auto indexed_obstacles = path_decision.obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
```

`SortObstaclesForSweepLine`函数

1. 遍历所有障碍物

   * 跳过虚拟障碍物
   * 跳过横向与纵向都是忽略决策的障碍物
   * 跳过动态障碍物

   ```C++
       if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {//只考虑非虚拟的静态障碍物（且没有ignore决策）
         continue;
       }
   ```

   ```C++
   bool IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
     // Obstacle should be non-virtual.
     if (obstacle.IsVirtual()) {
       return false;
     }
     // Obstacle should not have ignore decision.
     if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
         obstacle.IsIgnore()) {
       return false;
     }
     // Obstacle should not be moving obstacle.
     if (!obstacle.IsStatic() ||
         obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {//0.5
       return false;
     }
     // TODO(jiacheng):
     // Some obstacles are not moving, but only because they are waiting for
     // red light (traffic rule) or because they are blocked by others (social).
     // These obstacles will almost certainly move in the near future and we
     // should not side-pass such obstacles.
   
     return true;
   }
   ```

   * 跳过在adc后面的障碍物

   ```C++
       if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {//障碍物在车后无影响
         continue;
       }
   ```

   * 根据之前计算的障碍物的sl_boundary，首先对其进行膨胀，前2m，后3m,左右分别为0.4m,并将障碍物的首尾放入sorted_obstacles中

   ```C++
       const auto obstacle_sl = obstacle->PerceptionSLBoundary();
       sorted_obstacles.emplace_back(
           1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,//3
           obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,//0.4
           obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
       sorted_obstacles.emplace_back(
           0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,//2
           obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,//0.4
           obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
   ```

2. 对处理后的障碍物进行排序，排序规则

   * 先按照s值进行从小到大排序
   * 若是s值相同，则起始点在终止点前面

   ```C++
     std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
               [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
                 if (std::get<1>(lhs) != std::get<1>(rhs)) {
                   return std::get<1>(lhs) < std::get<1>(rhs);
                 } else {
                   return std::get<0>(lhs) > std::get<0>(rhs);
                 }
               });
   ```

* 遍历每个path_boundary中的s点，对每个点分别调整

  * 得到当前点的s值

  ```C++
  double curr_s = std::get<0>((*path_boundaries)[i]);
  ```

  * 如果所有的障碍物均在该点之前，则不对该点的s值进行调整，并更新中心点

  ```C++
   // If no obstacle change, update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) =
            std::fmax(std::get<1>((*path_boundaries)[i]),
                      *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) =
            std::fmin(std::get<2>((*path_boundaries)[i]),
                      *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }
  ```

  * 反之如果有障碍物在该点之前，则遍历所有在该点之前的起始点障碍物，判断该障碍物在中心点的左边还是右边，如果在左边则将障碍物的start_l作为边界的左boundary,如果是在右边，则将障碍物的end_l作为边界的右边界，之后更新中心线

  ```C++
      if (obs_idx < sorted_obstacles.size() &&
          std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        while (obs_idx < sorted_obstacles.size() &&
               std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
          const auto& curr_obstacle = sorted_obstacles[obs_idx];
          const double curr_obstacle_s = std::get<1>(curr_obstacle);
          const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
          const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
          const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
          ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
                 << "] curr_obstacle_l_min[" << curr_obstacle_l_min
                 << "] curr_obstacle_l_max[" << curr_obstacle_l_max
                 << "] center_line[" << center_line << "]";
          if (std::get<0>(curr_obstacle) == 1) {
            // A new obstacle enters into our scope:
            //   - Decide which direction for the ADC to pass.
            //   - Update the left/right bound accordingly.
            //   - If boundaries blocked, then decide whether can side-pass.
            //   - If yes, then borrow neighbor lane to side-pass.
            if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {//障碍物在引导线右边
              // Obstacle is to the right of center-line, should pass from left.
              obs_id_to_direction[curr_obstacle_id] = true;
              right_bounds.insert(curr_obstacle_l_max);//则右边边界为该障碍物的l_max
            } else {
              // Obstacle is to the left of center-line, should pass from right.
              obs_id_to_direction[curr_obstacle_id] = false;
              left_bounds.insert(curr_obstacle_l_min);//反之在左边，因此设置左边边界为l_min
            }
            if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              *blocking_obstacle_id = curr_obstacle_id;
              break;
            }
          } else {
            // An existing obstacle exits our scope.
            if (obs_id_to_direction[curr_obstacle_id]) {
              right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
            } else {
              left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
            }
            obs_id_to_direction.erase(curr_obstacle_id);
          }
          // Update the bounds and center_line.
          std::get<1>((*path_boundaries)[i]) = std::fmax(
              std::get<1>((*path_boundaries)[i]),
              *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
          std::get<2>((*path_boundaries)[i]) = std::fmin(
              std::get<2>((*path_boundaries)[i]),
              *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
          if (std::get<1>((*path_boundaries)[i]) >
              std::get<2>((*path_boundaries)[i])) {
            ADEBUG << "Path is blocked at s = " << curr_s;
            path_blocked_idx = static_cast<int>(i);
            if (!obs_id_to_direction.empty()) {
              *blocking_obstacle_id = obs_id_to_direction.begin()->first;
            }
            break;
          } else {
            center_line = (std::get<1>((*path_boundaries)[i]) +
                           std::get<2>((*path_boundaries)[i])) /
                          2.0;
          }
  
          ++obs_idx;
        }
  ```

* 最后根据阻挡的idx修剪path_boundary

```C++
  TrimPathBounds(path_blocked_idx, path_boundaries);
```

