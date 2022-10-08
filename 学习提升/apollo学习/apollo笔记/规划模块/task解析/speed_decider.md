<center><span style="font-size:2rem;font-weight:bold;">speed_decider解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

对纵向没有决策的障碍物添加决策信息





# 函数处理

* 遍历每个障碍物

  * 获得当前障碍物的boundary

  ```C++
      auto* mutable_obstacle = path_decision->Find(obstacle->Id());
      const auto& boundary = mutable_obstacle->path_st_boundary();
  ```

  * 如果该障碍物不存在boundary或者在adc的后面或者超出了轨迹总时间，则跳过

  ```C++
      if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
          boundary.max_t() < 0.0 ||
          boundary.min_t() >= speed_profile.back().t()) {
        AppendIgnoreDecision(mutable_obstacle);
        continue;
      }//障碍物对我们没影响，忽略它
  ```

  * 如果该障碍物存在纵向决策，则跳过

  ```C++
      if (obstacle->HasLongitudinalDecision()) {//对障碍物的纵向，横向决策进行判定，若是没有则添加忽略决策
        AppendIgnoreDecision(mutable_obstacle);
        continue;
      }
  ```

  * 跳过不在lane上的虚拟障碍物

  ```C++
      // for Virtual obstacle, skip if center point NOT "on lane"
      if (obstacle->IsVirtual()) {//跳过不在引导线上的虚拟障碍物
        const auto& obstacle_box = obstacle->PerceptionBoundingBox();
        if (!reference_line_->IsOnLane(obstacle_box.center())) {
          continue;
        }
      }
  ```

  * 如果该障碍物的类型是pedestrian，则判断是生成stop_wall

  ```C++
      if (CheckStopForPedestrian(*mutable_obstacle)) {//根据障碍物类型判断,如果还未停车，则为true，反之已经停车超过4s,则为false
        ObjectDecisionType stop_decision;
        if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                               -FLAGS_min_stop_distance_obstacle)) {//6,根据车头目前的s，行人最小s与FLAGS_min_stop_distance_obstacle计算停车点
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                    stop_decision);
        }
        continue;
      }
  ```

  判断依据：

  * **若是行人与path相交的点与adc相距大于10m则一定生成**
  * **若是小于10m，则判断行人速度是否大于0.3m/s，若是则一定生成**
  * **多是小于0.3m/s,则检测停车时间是否大于4s，若是则不生成**

  ```C++
    if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer) {//如果行人在车前10m内
      const auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                             perception_obstacle.velocity().y());//计算此时人的速度
      if (obstacle_speed > kMaxStopSpeed) {//行人速度大于0.3,result==true
        stop_time_map.erase(obstacle_id);
      } else {//行人速度小于0.3
        if (stop_time_map.count(obstacle_id) == 0) {
          // add timestamp
          stop_time_map[obstacle_id] = Clock::NowInSeconds();//设置停车时间为当前时间
          ADEBUG << "add timestamp: obstacle_id[" << obstacle_id << "] timestamp["
                 << Clock::NowInSeconds() << "]";
        } else {
          // check timeout
          double stop_timer = Clock::NowInSeconds() - stop_time_map[obstacle_id];//已经停了的时间
          ADEBUG << "stop_timer: obstacle_id[" << obstacle_id << "] stop_timer["
                 << stop_timer << "]";
          if (stop_timer >= kPedestrianStopTimeout) {//如果已经停了的时间大于4s,则返回false
            result = false;
          }
        }
      }
    }
  ```

  * CreateStopDecision函数

  在行人前6m处设置stop_wall

  ```C++
  bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                        ObjectDecisionType* const stop_decision,
                                        double stop_distance) const {
    const auto& boundary = obstacle.path_st_boundary();//
  
    // TODO(all): this is a bug! Cannot mix reference s and path s!
    // Replace boundary.min_s() with computed reference line s
    // fence is set according to reference line s.
    double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;
    if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      fence_s = obstacle.PerceptionSLBoundary().start_s();
    }
    const double main_stop_s =
        reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < fence_s) {
      ADEBUG << "Stop fence is further away, ignore.";
      return false;
    }
  
    const auto fence_point = reference_line_->GetReferencePoint(fence_s);
  
    // set STOP decision
    auto* stop = stop_decision->mutable_stop();
    stop->set_distance_s(stop_distance);
    auto* stop_point = stop->mutable_stop_point();
    stop_point->set_x(fence_point.x());
    stop_point->set_y(fence_point.y());
    stop_point->set_z(0.0);
    stop->set_stop_heading(fence_point.heading());
  
    if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
    }
  
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    ADEBUG << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";
  
    return true;
  }
  ```

  * 之后判断该障碍物与adc之间的位置关系

  判断依据

  遍历speed_data中的每个点，利用叉乘的性质判断该障碍物处于s的上方还是下方

  ```C++\
  SpeedDecider::STLocation SpeedDecider::GetSTLocation(
      const PathDecision* const path_decision, const SpeedData& speed_profile,
      const STBoundary& st_boundary) const {
    if (st_boundary.IsEmpty()) {
      return BELOW;
    }
  
    STLocation st_location = BELOW;
    bool st_position_set = false;
    const double start_t = st_boundary.min_t();
    const double end_t = st_boundary.max_t();
    for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {//遍历速度path的每个st点
      const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
      const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());
      if (curr_st.t() < start_t && next_st.t() < start_t) {//这个时刻与障碍物无交集
        continue;
      }
      if (curr_st.t() > end_t) {//同理无交集
        break;
      }
  
      if (!FLAGS_use_st_drivable_boundary) {
        common::math::LineSegment2d speed_line(curr_st, next_st);
        if (st_boundary.HasOverlap(speed_line)) {
          ADEBUG << "speed profile cross st_boundaries.";
          st_location = CROSS;
  
          if (!FLAGS_use_st_drivable_boundary) {
            if (st_boundary.boundary_type() ==
                STBoundary::BoundaryType::KEEP_CLEAR) {
              if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                           st_boundary)) {
                st_location = BELOW;
              }
            }
          }
          break;
        }
      }
  
      // note: st_position can be calculated by checking two st points once
      //       but we need iterate all st points to make sure there is no CROSS
      if (!st_position_set) {
        if (start_t < next_st.t() && curr_st.t() < end_t) {//障碍物与此时间段有交集
          STPoint bd_point_front = st_boundary.upper_points().front();
          double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
          st_location = side < 0.0 ? ABOVE : BELOW;
          st_position_set = true;
        }
      }
    }//path点遍历结束
    return st_location;
  }
  ```

  * 如果障碍物在adc的前面

    * 首先判断是否是跟随状态
      判断条件

      //障碍物与路径横向距离大于2.5m则不是跟随

      //障碍物朝车的方向运动，则不是跟随

      //如果障碍物在车前的时间小于2.0s则不会跟随

      ```C++
              else if (CheckIsFollow(*obstacle, boundary)) {//检测车否在障碍物后面跟随
                // stop for low_speed decelerating
                //即只会计算原本在同一车道的障碍物
                //障碍物的速度大于车的速度，则不会距离太近
                //当车速度大于障碍物速度时，若是车减速到障碍物的速度时与障碍物间的距离小于6m时，判断太近，需要停止
                if (IsFollowTooClose(*mutable_obstacle)) {//跟随距离太近，停车
                  ObjectDecisionType stop_decision;
                  if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                         -FLAGS_min_stop_distance_obstacle)) {
                    mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                              stop_decision);
                  }
                } else {  // high speed or low speed accelerating
                  // FOLLOW decision
                  ObjectDecisionType follow_decision;
                  if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {//计算跟随点，设定跟随决策
                    mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                              follow_decision);
                  }
                }
              } 
      ```

      * 如果是跟随状态则判断是否跟随太近

      判断条件

      即只会计算原本在同一车道的障碍物
      障碍物的速度大于车的速度，则不会距离太近
      当车速度大于障碍物速度时，若是车减速到障碍物的速度时与障碍物间的距离小于6m时，判断太近，需要停止

      * 如果跟随太近则会生成stop的决策
      * 反之则会生成follow决策
      * 如果不是跟随状态，则生成yield决策

  * 车在障碍物后面，则生成超车决策