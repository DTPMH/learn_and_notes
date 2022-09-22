<center><span style="font-size:2rem;font-weight:bold;">lane_borrow_decider</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总体流程

* 首先默认设置为不借道
  ```C++
    reference_line_info->set_is_path_lane_borrow(false);
  ```

* 之后读取规划的配置，若是配置中允许借道且此时有必要借道，则将该引导线借道值赋为`true`
  ```C++
    if (Decider::config_.path_lane_borrow_decider_config()
            .allow_lane_borrowing() &&//true
        IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
      reference_line_info->set_is_path_lane_borrow(true);
    }
  ```

# `IsNecessaryToBorrowLane`函数

* 首先判断之前的状态，若是上一帧就是借道的状态，则判断使用`regular/self`路径的帧数，若是连续超过5帧使用了`regular/self`的path，则将借道状态复位。
  ```C++
   if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6) {//如果自己能够使用的lane超过6，则改变状态
        // If have been able to use self-lane for some time, then switch to
        // non-lane-borrowing.
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        mutable_path_decider_status->clear_decided_side_pass_direction();
        AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
      }
  ```

* 若是上一帧的状态不是借道，则判断是否要借道，判断条件如下：

  * 只有一条引导线
    ```C++
        if (!HasSingleReferenceLine(frame)) {//如果有多条参考线，则返回false（即属于change_lane），只有一条参考线才可以借道
          return false;
        }
    ```

    ```C++
    bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
      return frame.reference_line_info().size() == 1;
    }
    ```

  * `adc`的速度小于`5m/s`

    ```C++
        if (!IsWithinSidePassingSpeedADC(frame)) {//开始规划点的速度小于5m/s才可以借道
          return false;
        }
    ```

    ```C++
    bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
      return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed;//5.0
    }
    ```

  * 障碍物周围20m范围内不存在`signal`与`stop_sign`区域
    ```c++
        if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {//判断障碍物是否远离参考线中的特殊区域，远离true
          return false;
        }
    ```

    ```c++
    bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
        const ReferenceLineInfo& reference_line_info) {
      const auto& path_decider_status =
          injector_->planning_context()->planning_status().path_decider();
      const std::string blocking_obstacle_id =
          path_decider_status.front_static_obstacle_id();//（在assessment中赋值）
      if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
      }
      const Obstacle* blocking_obstacle =
          reference_line_info.path_decision().obstacles().Find(
              blocking_obstacle_id);
      if (blocking_obstacle == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
      }
    
      // Get blocking obstacle's s.
      double blocking_obstacle_s =
          blocking_obstacle->PerceptionSLBoundary().end_s();
      ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
      // Get intersection's s and compare with threshold.
      const auto& first_encountered_overlaps =
          reference_line_info.FirstEncounteredOverlaps();
      for (const auto& overlap : first_encountered_overlaps) {//遍历特殊区域
        ADEBUG << overlap.first << ", " << overlap.second.DebugString();
        // if (// overlap.first != ReferenceLineInfo::CLEAR_AREA &&
        // overlap.first != ReferenceLineInfo::CROSSWALK &&
        // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
        if (overlap.first != ReferenceLineInfo::SIGNAL &&
            overlap.first != ReferenceLineInfo::STOP_SIGN) {
          continue;
        }
    
        auto distance = overlap.second.start_s - blocking_obstacle_s;
        if (overlap.first == ReferenceLineInfo::SIGNAL ||
            overlap.first == ReferenceLineInfo::STOP_SIGN) {
          if (distance < kIntersectionClearanceDist) {
            ADEBUG << "Too close to signal intersection (" << distance
                   << "m); don't SIDE_PASS.";
            return false;
          }
        } else {
          if (distance < kJunctionClearanceDist) {
            ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
                   << distance << "m); don't SIDE_PASS";
            return false;
          }
        }
      }//遍历车道特殊区域结束
    
      return true;
    }
    ```

  * 该障碍物存在周期大于3个计算周期
    ```C++
        if (!IsLongTermBlockingObstacle()) {//
          return false;
        }
    ```

    ```c++
    bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
      if (injector_->planning_context()
              ->planning_status()
              .path_decider()
              .front_static_obstacle_cycle_counter() >=
          FLAGS_long_term_blocking_obstacle_cycle_threshold) {//3
        ADEBUG << "The blocking obstacle is long-term existing.";
        return true;
      } else {
        ADEBUG << "The blocking obstacle is not long-term existing.";
        return false;
      }
    }
    ```

  * 障碍物在终点前
    ```C++
        if (!IsBlockingObstacleWithinDestination(reference_line_info)) {//如果目标点有障碍物
          return false;
        }
    ```

    ```c++
    bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
        const ReferenceLineInfo& reference_line_info) {
      const auto& path_decider_status =
          injector_->planning_context()->planning_status().path_decider();
      const std::string blocking_obstacle_id =
          path_decider_status.front_static_obstacle_id();
      if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
      }
      const Obstacle* blocking_obstacle =
          reference_line_info.path_decision().obstacles().Find(
              blocking_obstacle_id);
      if (blocking_obstacle == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
      }
    
      double blocking_obstacle_s =
          blocking_obstacle->PerceptionSLBoundary().start_s();
      double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
      ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
      ADEBUG << "ADC is at s = " << adc_end_s;
      ADEBUG << "Destination is at s = "
             << reference_line_info.SDistanceToDestination() + adc_end_s;
      if (blocking_obstacle_s - adc_end_s >
          reference_line_info.SDistanceToDestination()) {
        return false;
      }
      return true;
    }
    ```

  * `isSidePassableObstacle`函数

    * 首先判断该障碍物是否存在，存在则继续判断该障碍物是否为不动的障碍物，不存在则返回false，不需要借道
      ```C++
        const auto& path_decider_status =
            injector_->planning_context()->planning_status().path_decider();
        const std::string blocking_obstacle_id =
            path_decider_status.front_static_obstacle_id();
        if (blocking_obstacle_id.empty()) {
          ADEBUG << "There is no blocking obstacle.";
          return false;
        }
        const Obstacle* blocking_obstacle =
            reference_line_info.path_decision().obstacles().Find(
                blocking_obstacle_id);
        if (blocking_obstacle == nullptr) {
          ADEBUG << "Blocking obstacle is no longer there.";
          return false;
        }
      ```

    * 之后判断该障碍物是否是不动的障碍物，是则返回true，需要借道，否则返回false，不需要借道
      **`IsNonmovableObstacle`函数见[下面](#`IsNonmovableObstacle`函数)分析**
      
      ```C++
      return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);//是否为不移动的障碍物
      ```
  
  * 开始检查道路状态是否可以借道，以`2m`为分辨率检查车前100m的范围内是否存在不可以借道的lane（左边或者右边为黄线或者为实线）,检查完毕若是存在可以借道的lane，则返回true，可以借道，否则不可以借道。
    ```C++
        const auto& path_decider_status =
            injector_->planning_context()->planning_status().path_decider();
        if (path_decider_status.decided_side_pass_direction().empty()) {
          // first time init decided_side_pass_direction
          bool left_borrowable;
          bool right_borrowable;
          CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);//检查邻居变道是否可以变道
          if (!left_borrowable && !right_borrowable) {
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
            return false;
          } else {
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
            if (left_borrowable) {
              mutable_path_decider_status->add_decided_side_pass_direction(
                  PathDeciderStatus::LEFT_BORROW);
            }
            if (right_borrowable) {
              mutable_path_decider_status->add_decided_side_pass_direction(
                  PathDeciderStatus::RIGHT_BORROW);
            }
          }
        }
    ```

    * `CheckLaneBorrow`函数
  
      ```C++
      void PathLaneBorrowDecider::CheckLaneBorrow(
          const ReferenceLineInfo& reference_line_info,
          bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
        const ReferenceLine& reference_line = reference_line_info.reference_line();
      
        *left_neighbor_lane_borrowable = true;
        *right_neighbor_lane_borrowable = true;
      
        static constexpr double kLookforwardDistance = 100.0;
        double check_s = reference_line_info.AdcSlBoundary().end_s();
        const double lookforward_distance =
            std::min(check_s + kLookforwardDistance, reference_line.Length());
        while (check_s < lookforward_distance) {
          auto ref_point = reference_line.GetNearestReferencePoint(check_s);
          if (ref_point.lane_waypoints().empty()) {
            *left_neighbor_lane_borrowable = false;
            *right_neighbor_lane_borrowable = false;
            return;
          }
      
          const auto waypoint = ref_point.lane_waypoints().front();
          hdmap::LaneBoundaryType::Type lane_boundary_type =
              hdmap::LaneBoundaryType::UNKNOWN;
      
          if (*left_neighbor_lane_borrowable) {
            lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
                lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
              *left_neighbor_lane_borrowable = false;
            }
            ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
                   << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
          }
          if (*right_neighbor_lane_borrowable) {
            lane_boundary_type = hdmap::RightBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
                lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
              *right_neighbor_lane_borrowable = false;
            }
            ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
                   << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
          }
          check_s += 2.0;
        }
      }
      ```
  
      

# `IsNonmovableObstacle`函数

* 首先判断此时障碍物与车之间的距离，若是朝过35m，则返回false
  ```C++
    const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
    if (obstacle.PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
      ADEBUG << " - It is too far ahead and we are not so sure of its status.";
      return false;
    }
  ```

* 之后判断该车是否为停在路边的车，判断依据为此时障碍物的sl_boundary与road的边界差值是否在0.1内,且该障碍物是静态障碍物
  ```C++
    if (IsParkedVehicle(reference_line_info.reference_line(), &obstacle)) {
      ADEBUG << "It is Parked and NON-MOVABLE.";
      return true;
    }
  ```

  ```C++
  bool IsParkedVehicle(const ReferenceLine& reference_line,
                       const Obstacle* obstacle) {
    if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
      return false;
    }
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    double max_road_right_width = 0.0;
    reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                                &road_left_width, &road_right_width);
    max_road_right_width = road_right_width;
    reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                                &road_left_width, &road_right_width);
    max_road_right_width = std::max(max_road_right_width, road_right_width);
    bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                           max_road_right_width - 0.1;
  
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    auto obstacle_box = obstacle->PerceptionBoundingBox();
    HDMapUtil::BaseMapPtr()->GetLanes(
        common::util::PointFactory::ToPointENU(obstacle_box.center().x(),
                                               obstacle_box.center().y()),
        std::min(obstacle_box.width(), obstacle_box.length()), &lanes);
    bool is_on_parking_lane = false;
    if (lanes.size() == 1 &&
        lanes.front()->lane().type() == apollo::hdmap::Lane::PARKING) {
      is_on_parking_lane = true;
    }
  
    bool is_parked = is_on_parking_lane || is_at_road_edge;
    return is_parked && obstacle->IsStatic();
  }
  ```

* 之后遍历所有的其他障碍物，查看该障碍物前面15m范围内是否存在其他障碍物阻挡(判断依据：两个障碍物之间的sl_boundary,主要是l是否有阻挡)
  ```C++
    for (const auto* other_obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (other_obstacle->Id() == obstacle.Id()) {
        continue;
      }
      if (other_obstacle->IsVirtual()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              obstacle.PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              obstacle.PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }
      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       obstacle.PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kObstaclesDistanceThreshold) {
        continue;
      }
  
      // TODO(All): Fix the segmentation bug for large vehicles, otherwise
      // the follow line will be problematic.
      ADEBUG << " - It is blocked by others, and will move later.";
      return false;
    }
  ```

  