<center><span style="font-size:2rem;font-weight:bold;">path_decider解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

path_decider是对静态障碍物进行分析，根据静态障碍物的类型与所处位置，构建虚拟墙与相应的决策

# 函数流程

* 首先遍历所有的障碍物

  * 跳过动态障碍物和虚拟障碍物

  ```C++
      if (!obstacle->IsStatic() || obstacle->IsVirtual()) {//非静态障碍物与虚拟障碍物不考虑
        continue;
      }
  ```

  * 跳过横纵向决策均为忽略的障碍物

  ```C++
      if (obstacle->HasLongitudinalDecision() &&
          obstacle->LongitudinalDecision().has_ignore() &&
          obstacle->HasLateralDecision() &&
          obstacle->LateralDecision().has_ignore()) {
        continue;
      }
  ```

  * 跳过纵向决策为停止的障碍物

  ```C++
      if (obstacle->HasLongitudinalDecision() &&
          obstacle->LongitudinalDecision().has_stop()) {//纵向停止则跳过
        // STOP decision
        continue;
      }
  ```

  * 如果该障碍物就是上一步检测到的阻挡lane的障碍物，则生成stop决策，并为障碍物添加纵向决策信息,设置stop_reason为：stop for obstacle

  ```C++
      if (obstacle->Id() == blocking_obstacle_id &&
          !injector_->planning_context()
               ->planning_status()
               .path_decider()
               .is_in_path_lane_borrow_scenario()) {//如果就是这个阻挡的障碍物且不处于borrowlane场景
        // Add stop decision
        ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
        ObjectDecisionType object_decision;
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
        //根据障碍物在引导线上的起点与停车距离设定停车点
        path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                               obstacle->Id(), object_decision);
        continue;
      }
  ```

  ```C++
  ObjectStop PathDecider::GenerateObjectStopDecision(
      const Obstacle &obstacle) const {
    ObjectStop object_stop;
  
    double stop_distance = obstacle.MinRadiusStopDistance(
        VehicleConfigHelper::GetConfig().vehicle_param());//停车最小距离
    object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
    object_stop.set_distance_s(-stop_distance);
  
    const double stop_ref_s =
        obstacle.PerceptionSLBoundary().start_s() - stop_distance;
    const auto stop_ref_point =
        reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);//得到参考线停车点
    object_stop.mutable_stop_point()->set_x(stop_ref_point.x());//停车点坐标
    object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
    object_stop.set_stop_heading(stop_ref_point.heading());
    return object_stop;
  }
  ```

  * 跳过kc区域

  ```C++
      if (obstacle->reference_line_st_boundary().boundary_type() ==
          STBoundary::BoundaryType::KEEP_CLEAR) {
        continue;
      }
  ```

  * 如果障碍物在path之前或者在path最后一个点的后面，则设置忽略决策

  ```C++
      ObjectDecisionType object_decision;//障碍物不在引导线上
      object_decision.mutable_ignore();
      const auto &sl_boundary = obstacle->PerceptionSLBoundary();
      if (sl_boundary.end_s() < frenet_path.front().s() ||
          sl_boundary.start_s() > frenet_path.back().s()) {
        path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                               obstacle->Id(), object_decision);
        path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                          object_decision);
        continue;
      }
  ```

  * 如果障碍物与最近的path点之间的横向距离超过3m(3m+0.5车宽)，则设置横向忽略决策

  ```C++
      if (curr_l - lateral_radius > sl_boundary.end_l() ||//
          curr_l + lateral_radius < sl_boundary.start_l()) {
        // 1. IGNORE if laterally too far away.
        path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),////将障碍物的id与decision放入path_decision中
                                          object_decision);
      }
  ```

  * 如果障碍物在车身宽度加上0.15的范围内，则设置人横纵向stop决策

  ```C++
  else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
                 sl_boundary.start_l() <= curr_l + min_nudge_l) {
        // 2. STOP if laterally too overlapping.
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
  
        if (path_decision->MergeWithMainStop(
                object_decision.stop(), obstacle->Id(),
                reference_line_info_->reference_line(),
                reference_line_info_->AdcSlBoundary())) {
          path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                                 obstacle->Id(), object_decision);
        } else {
          ObjectDecisionType object_decision;
          object_decision.mutable_ignore();
          path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                                 obstacle->Id(), object_decision);
        }
  ```

  * 其他情况则设置nudge决策，如果障碍物在车右边，则左nudge,反之左nudge

  ```C++
  else {
        // 3. NUDGE if laterally very close.
        if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
          // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {//障碍物在他的右边，需要左推
          // LEFT_NUDGE
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
          object_nudge_ptr->set_distance_l(
              config_.path_decider_config().static_obstacle_buffer());
          path_decision->AddLateralDecision("PathDecider/left-nudge",
                                            obstacle->Id(), object_decision);
        } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
          // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
          // RIGHT_NUDGE
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
          object_nudge_ptr->set_distance_l(
              -config_.path_decider_config().static_obstacle_buffer());
          path_decision->AddLateralDecision("PathDecider/right-nudge",
                                            obstacle->Id(), object_decision);
        }
      }
  ```

* 障碍物遍历结束