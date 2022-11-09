<center><span style="font-size:2rem;font-weight:bold;">lane_change_decider</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 主要流程

* 首先获取lane_change_decider的`config`
  ```c++
  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  ```

* 之后获取当前引导线信息
  ```C++
  std::list<ReferenceLineInfo>* reference_line_info = frame->mutable_reference_line_info();
  ```

* 之后获取上一帧规划的lane_change的状态
  ```C++
  auto* prev_status = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_change_lane();//一共有三种状态，正在变道，变道成功，变道失败
  ```

  **注意：第一帧状态是空的，因此在规划第一帧时会将状态置为变道成功(`CHANGE_LANE_FINISHED`)**

* 若当前引导线是变道的引导线，则根据目前是否能安全变道，对`prev_status->set_is_clear_to_change_lane`赋值，在path_bound中会用到
  ```C++
    if (current_reference_line_info->IsChangeLanePath()) {
      prev_status->set_is_clear_to_change_lane(
          IsClearToChangeLane(current_reference_line_info));//检测车在变道时是否能安全变道（同向最低10m,反向50m(需要根据速度计算)）
    }
  ```

  **`IsClearToChangeLane`函数内容见 [下面](# IsClearTochangeLane)分析**

* 如果当前引导线的数量等于`1`
  则说明目前不会变道，因此对此时状态进行判断：

  * 若上一帧状态为变道完成，则无处理(变道结束)

    ```C++
     if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
          //若只有一条参考线，且上一步处于变道中，则说明变道结束
        }
    ```

  * 若是上一帧状态处于变道状态，则说明变道结束
    ```C++
    else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
          UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, path_id);
    ```

  * 若是上一帧状态处于变道失败的状态，则无处理
    ```C++
    else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
        }
    ```

  * 否则，返回规划错误。
    ```C++
          const std::string msg =
              absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
    ```

* 如果引导线数量大于1，则说明正在变道

  * 获得当前道路的id
    ```C++
    auto current_path_id = GetCurrentPathId(*reference_line_info);
    ```

  * 如果当前道路id为空，则返回规划错误
    ```C++
        if (current_path_id.empty()) {
          const std::string msg = "The vehicle is not on any reference line";
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    ```

  * 如果上一个时刻正在变道，则比较上一次变道的id与此次道路的id是否相同，则说明还在变道中：
    ```C++
        if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
          //则比较上一次变道的id与当前变道的id,若是相同，则说明还在变道中
          if (prev_status->path_id() == current_path_id) {
            PrioritizeChangeLane(true, reference_line_info);//
          } 
    ```

  * 若是id不相同，则说明变道已经成功
    ```C++
    else {//ID不相同则说明变道已经成功
            // RemoveChangeLane(reference_line_info);
            PrioritizeChangeLane(false, reference_line_info);//目前不做处理
            ADEBUG << "removed change lane.";
            UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                         current_path_id);
          }
    ```

  * 如果上一步状态是变道失败的的话，计算冷冻时间
    ```C++
          // change_lane_failed status
          if (now - prev_status->timestamp() <
              lane_change_decider_config.change_lane_fail_freeze_time()) {
            // RemoveChangeLane(reference_line_info);
            PrioritizeChangeLane(false, reference_line_info);
            ADEBUG << "freezed after failed";
          } else {
            UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);//冷冻时间已过，重新修改状态
            ADEBUG << "change lane again after failed";
          }
    ```

  * 如果上一步状态是变道成功的话，则计算冷却时间
    ```C++
    else if (prev_status->status() ==
                   ChangeLaneStatus::CHANGE_LANE_FINISHED) {//上一时刻变道成功，但是还有两条参看线说明仍需变道
          if (now - prev_status->timestamp() <
              lane_change_decider_config.change_lane_success_freeze_time()) {
            // RemoveChangeLane(reference_line_info);
            PrioritizeChangeLane(false, reference_line_info);
            ADEBUG << "freezed after completed lane change";
          } else {
            PrioritizeChangeLane(true, reference_line_info);
            UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
            ADEBUG << "change lane again after success";
          }
        } 
    ```

  * 否则返回规划状态失败。

# IsClearTochangeLane

* 遍历所有障碍物

  ```c++
    for (const auto* obstacle :
         reference_line_info->path_decision()->obstacles().Items()) {//遍历这条参考线上所有的障碍物
  ```

* 跳过虚拟障碍物以及静态障碍物

  ```C++
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {//跳过虚拟障碍物与静态障碍物
        ADEBUG << "skip one virtual or static obstacle";
        continue;
      }
  ```

* 遍历障碍物Polygon点，计算其在引导线上的是s,l值，计算该障碍物最大范围的start_s，end_s,start_l，end_l。
  ```C++
      for (const auto& p : obstacle->PerceptionPolygon().points()) {
        SLPoint sl_point;
        reference_line_info->reference_line().XYToSL(p, &sl_point);
        start_s = std::fmin(start_s, sl_point.s());
        end_s = std::fmax(end_s, sl_point.s());
  
        start_l = std::fmin(start_l, sl_point.l());
        end_l = std::fmax(end_l, sl_point.l());
      }
  ```

* 如果该引导线是变道的，且该障碍物超出lane的范围，则跳过
  ```C++
      if (reference_line_info->IsChangeLanePath()) {//如果该参考线需要变道
        double left_width(0), right_width(0);
        reference_line_info->mutable_reference_line()->GetLaneWidth(
            (start_s + end_s) * 0.5, &left_width, &right_width);//得到障碍物中点处的边界宽
        if (end_l < -right_width || start_l > left_width) {//如果障碍物超出lane的边界范围，则跳过
          continue;
        }
      }
  ```

* 如果该障碍物有轨迹，则计算该障碍物与adc的速度方向

  ```C++
      if (obstacle->HasTrajectory()) {
        double obstacle_moving_direction =
            obstacle->Trajectory().trajectory_point(0).path_point().theta();//障碍物第一个点的朝向
        const auto& vehicle_state = reference_line_info->vehicle_state();
        double vehicle_moving_direction = vehicle_state.heading();
        if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
          vehicle_moving_direction =
              common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
        }
        double heading_difference = std::abs(common::math::NormalizeAngle(
            obstacle_moving_direction - vehicle_moving_direction));
        same_direction = heading_difference < (M_PI / 2.0);//小于90°说明在同一方向
      }
  ```

* 最后根据朝向是否一致计算障碍物与车之间的距离是否大于两者速度差在3s内的行驶范围，若大于则安全，否则不安全。
  ```C++
      static constexpr double kSafeTimeOnSameDirection = 3.0;
      static constexpr double kSafeTimeOnOppositeDirection = 5.0;
      static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
      static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
      static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
      static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
      static constexpr double kDistanceBuffer = 0.5;
  
      double kForwardSafeDistance = 0.0;
      double kBackwardSafeDistance = 0.0;
      if (same_direction) {//如果是同一方向
      //(车的速度大于障碍区的速度)
        kForwardSafeDistance =//计算同方向上的最小前向距离（当障碍物在车前时，最小距离为两者在同一位置行驶3s后的距离差））
            std::fmax(kForwardMinSafeDistanceOnSameDirection,
                      (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      //车的速度小于障碍物的速度
        kBackwardSafeDistance =//同上
            std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                      (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
      } else {
        kForwardSafeDistance =
            std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                      (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
        kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
      }
  //判断障碍物是否满足安全距离
      if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                           kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
          HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                           kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
        reference_line_info->path_decision()
            ->Find(obstacle->Id())
            ->SetLaneChangeBlocking(true);
        ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
        return false;
      } else {
        reference_line_info->path_decision()
            ->Find(obstacle->Id())
            ->SetLaneChangeBlocking(false);
      }
    }
  ```

  