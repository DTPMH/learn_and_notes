<center><span style="font-size:2rem;font-weight:bold;">path_assessment_decider解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

path_assessment_decider的作用是对path_data进行筛选，根据排序规则选择最优的path

# 函数流程

* 首先对path进行筛选，若是fallback的path检测其与中心线距离是否超过20m,regular的path检测其与中心线是否超过10m

```C++
  std::vector<PathData> valid_path_data;
  for (const auto& curr_path_data : candidate_path_data) {//首先找到合法path
    // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
    //                 reference_line_info);
    if (curr_path_data.path_label().find("fallback") != std::string::npos) {
      if (IsValidFallbackPath(*reference_line_info, curr_path_data)) {//path点远离中心线超过20m
        valid_path_data.push_back(curr_path_data);
      }
    } else {
      if (IsValidRegularPath(*reference_line_info, curr_path_data)) {//超出10m
        valid_path_data.push_back(curr_path_data);
      }
    }
  }//挑选结束
```

* 之后为非fallback的path里的每一个点打上标签,并修剪path使每一个path的最后一个点一定是inlane的

```C++
  const Obstacle* blocking_obstacle_on_selflane = nullptr;
  for (size_t i = 0; i != valid_path_data.size(); ++i) {
    auto& curr_path_data = valid_path_data[i];
    if (curr_path_data.path_label().find("fallback") != std::string::npos) {//fallback类型的path
      // remove empty path_data.
      if (!curr_path_data.Empty()) {//非空
        if (cnt != i) {
          valid_path_data[cnt] = curr_path_data;
        }
        ++cnt;
      }
      continue;//不对fallback轨迹进行以下处理
    }//fallback类型结束，
    SetPathInfo(*reference_line_info, &curr_path_data);//为curr_path_data上每个点打上标签（in_lane,out_to_forward_lane....）
    // Trim all the lane-borrowing paths so that it ends with an in-lane
    // position.
    if (curr_path_data.path_label().find("pullover") == std::string::npos) {//如果不是靠边停车的path，则修剪掉
      TrimTailingOutLanePoints(&curr_path_data);//修剪path_data使得最后一个点的decision为in_lane(不修剪pull_over，fall_back以及self)
    }//保证每条轨迹的最后一给点必须为in_lane类型

    // find blocking_obstacle_on_selflane, to be used for lane selection later
    if (curr_path_data.path_label().find("self") != std::string::npos) {//如果类型是self的
      const auto blocking_obstacle_id = curr_path_data.blocking_obstacle_id();
      blocking_obstacle_on_selflane =
          reference_line_info->path_decision()->Find(blocking_obstacle_id);
    }

    // remove empty path_data.
    if (!curr_path_data.Empty()) {
      if (cnt != i) {
        valid_path_data[cnt] = curr_path_data;
      }
      ++cnt;
    }

    // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
    //                 reference_line_info);
    ADEBUG << "For " << curr_path_data.path_label() << ", "
           << "path length = " << curr_path_data.frenet_frame_path().size();
  }//合法path遍历结束，添加path_decision（point_type）
```

* 之后对有效的path 进行排序，选择第一个path作为最终的path
  排序规则：

  * 优先选择regular的path
  * 选择最长的path
  * 如果两者中存在self路径，且若是两者长度差值大于15m则选择更长的，否则选择self路径
  * 如果两者都不是self路径,且两者长度差值大于25m，则选择更长的
  * 长度差不多，则选择同向借道的
  * 等等

  ```C++
  bool ComparePathData(const PathData& lhs, const PathData& rhs,
                       const Obstacle* blocking_obstacle) {
    ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();
    // Empty path_data is never the larger one.
    if (lhs.Empty()) {
      ADEBUG << "LHS is empty.";
      return false;
    }
    if (rhs.Empty()) {
      ADEBUG << "RHS is empty.";
      return true;
    }
    // Regular path goes before fallback path.
    bool lhs_is_regular = lhs.path_label().find("regular") != std::string::npos;
    bool rhs_is_regular = rhs.path_label().find("regular") != std::string::npos;
    if (lhs_is_regular != rhs_is_regular) {
      return lhs_is_regular;
    }
    // Select longer path.
    // If roughly same length, then select self-lane path.
    bool lhs_on_selflane = lhs.path_label().find("self") != std::string::npos;
    bool rhs_on_selflane = rhs.path_label().find("self") != std::string::npos;
    static constexpr double kSelfPathLengthComparisonTolerance = 15.0;
    static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
    double lhs_path_length = lhs.frenet_frame_path().back().s();
    double rhs_path_length = rhs.frenet_frame_path().back().s();
    if (lhs_on_selflane || rhs_on_selflane) {
      if (std::fabs(lhs_path_length - rhs_path_length) >
          kSelfPathLengthComparisonTolerance) {
        return lhs_path_length > rhs_path_length;
      } else {
        return lhs_on_selflane;
      }
    } else {
      if (std::fabs(lhs_path_length - rhs_path_length) >
          kNeighborPathLengthComparisonTolerance) {
        return lhs_path_length > rhs_path_length;
      }
    }
    // If roughly same length, and must borrow neighbor lane,
    // then prefer to borrow forward lane rather than reverse lane.
    int lhs_on_reverse =
        ContainsOutOnReverseLane(lhs.path_point_decision_guide());
    int rhs_on_reverse =
        ContainsOutOnReverseLane(rhs.path_point_decision_guide());
    // TODO(jiacheng): make this a flag.
    if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
      return lhs_on_reverse < rhs_on_reverse;
    }
    // For two lane-borrow directions, based on ADC's position,
    // select the more convenient one.
    if ((lhs.path_label().find("left") != std::string::npos &&
         rhs.path_label().find("right") != std::string::npos) ||
        (lhs.path_label().find("right") != std::string::npos &&
         rhs.path_label().find("left") != std::string::npos)) {
      if (blocking_obstacle) {
        // select left/right path based on blocking_obstacle's position
        const double obstacle_l =
            (blocking_obstacle->PerceptionSLBoundary().start_l() +
             blocking_obstacle->PerceptionSLBoundary().end_l()) /
            2;
        ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l[" << obstacle_l
               << "]";
        return (obstacle_l > 0.0
                    ? (lhs.path_label().find("right") != std::string::npos)
                    : (lhs.path_label().find("left") != std::string::npos));
      } else {
        // select left/right path based on ADC's position
        double adc_l = lhs.frenet_frame_path().front().l();
        if (adc_l < -1.0) {
          return lhs.path_label().find("right") != std::string::npos;
        } else if (adc_l > 1.0) {
          return lhs.path_label().find("left") != std::string::npos;
        }
      }
    }
    // If same length, both neighbor lane are forward,
    // then select the one that returns to in-lane earlier.
    static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
    int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
    int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
    double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
    double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
    if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
      return lhs_back_idx < rhs_back_idx;
    }
    // If same length, both forward, back to inlane at same time,
    // select the left one to side-pass.
    bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
    bool rhs_on_leftlane = rhs.path_label().find("left") != std::string::npos;
    if (lhs_on_leftlane != rhs_on_leftlane) {
      ADEBUG << "Select " << (lhs_on_leftlane ? "left" : "right") << " lane over "
             << (!lhs_on_leftlane ? "left" : "right") << " lane.";
      return lhs_on_leftlane;
    }
    // Otherwise, they are the same path, lhs is not < rhs.
    return false;
  }
  ```

* 最后计算障碍物存在周期与使用的selfpath的次数，用于lane_borrow_decider

```C++
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter, 0));
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter + 1, 10));
    mutable_path_decider_status->set_front_static_obstacle_id(
        reference_line_info->GetBlockingObstacle()->Id());
  } else {
    int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::min(front_static_obstacle_cycle_counter, 0));
    mutable_path_decider_status->set_front_static_obstacle_cycle_counter(
        std::max(front_static_obstacle_cycle_counter - 1, -10));
  }

  // Update self-lane usage info.
  if (reference_line_info->path_data().path_label().find("self") !=
      std::string::npos) {
    // && std::get<1>(reference_line_info->path_data()
    //                 .path_point_decision_guide()
    //                 .front()) == PathData::PathPointType::IN_LANE)
    int able_to_use_self_lane_counter =
        mutable_path_decider_status->able_to_use_self_lane_counter();

    if (able_to_use_self_lane_counter < 0) {
      able_to_use_self_lane_counter = 0;
    }
    mutable_path_decider_status->set_able_to_use_self_lane_counter(
        std::min(able_to_use_self_lane_counter + 1, 10));
  } else {
    mutable_path_decider_status->set_able_to_use_self_lane_counter(0);
  }

```



