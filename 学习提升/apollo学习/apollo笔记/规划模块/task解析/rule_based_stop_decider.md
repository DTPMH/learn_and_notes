<center><span style="font-size:2rem;font-weight:bold;">rule_based_stop_decider解析</span></center>

# 总览

该task是判断是否需要紧急停车

# 函数流程

* 首先判断借道时是否需要停车

```C++
  //检查borrow时是否要停止
  StopOnSidePass(frame, reference_line_info);
```

```C++
void RuleBasedStopDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  static bool check_clear;
  static common::PathPoint change_lane_stop_path_point;

  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;

  if (path_data.path_label().find("self") != std::string::npos) {//SELF，直接返回
    check_clear = false;
    change_lane_stop_path_point.Clear();
    return;
  }

  if (check_clear &&
      CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
    check_clear = false;
  }

  if (!check_clear &&
      CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {//在想逆向车道变道时，计算stop_s_on_pathdata
    if (!LaneChangeDecider::IsPerceptionBlocked(
            *reference_line_info,
            rule_based_stop_decider_config_.search_beam_length(),//20
            rule_based_stop_decider_config_.search_beam_radius_intensity(),//0.08
            rule_based_stop_decider_config_.search_range(),//3.14
            rule_based_stop_decider_config_.is_block_angle_threshold()) &&//0.5检测范围内是否存在障碍物
        LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {//如果范围内不存在障碍物，且能够变道返回true
      return;
    }//检测变道前一个点是否有障碍物阻挡
    if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
      if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                  &change_lane_stop_path_point, frame,
                                  reference_line_info)) {
        AERROR << "Set side pass stop fail";
      }
    } else {
      if (LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
        check_clear = true;
      }
    }
  }
}
```

