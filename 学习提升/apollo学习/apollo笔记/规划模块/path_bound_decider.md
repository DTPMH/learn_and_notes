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

  `GenerateFallbackPathBound`函数详细[如下](#`GenerateFallbackPathBound`函数)
  
* 之后检查系统是否处于pull_over场景，若是处于则计算pull_over的path_bound,若是不处于则跳过
  ```C++
  
  ```

  



# 各个分函数详细介绍

## `InitPathBoundsDecider`函数





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

* 之后根据`adc`的位置和`lanes`计算`path_bound`
  代码：

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

  `GetBoundaryFromLanesAndADC`函数

  * 首先计算遍历初始化后的`path_bound`中的每一个点，计算其对应的path_bound
  
    * 根据当前点的s值，利用引导线计算当前lane的宽度与当前`road`的宽度，并将对应的path_bound根据引导线的`l`值进行平移
  
      ```C++
          if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                           &curr_lane_right_width)) {
            AWARN << "Failed to get lane width at s = " << curr_s;
            curr_lane_left_width = past_lane_left_width;
            curr_lane_right_width = past_lane_right_width;
          } else {
            // check if lane boundary is also road boundary
            double curr_road_left_width = 0.0;
            double curr_road_right_width = 0.0;
            if (reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                            &curr_road_right_width)) {
              is_left_lane_boundary =
                  (std::abs(curr_road_left_width - curr_lane_left_width) >
                   boundary_buffer);
              is_right_lane_boundary =
                  (std::abs(curr_road_right_width - curr_lane_right_width) >
                   boundary_buffer);
            }
            reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
            curr_lane_left_width += offset_to_lane_center;
            curr_lane_right_width -= offset_to_lane_center;
            past_lane_left_width = curr_lane_left_width;
            past_lane_right_width = curr_lane_right_width;
          }
      ```
      
    * 计算顺势横向速度位移(计算原因暂时不了解)
      ```C++
          static constexpr double kMaxLateralAccelerations = 1.5;
          double offset_to_map = 0.0;
          reference_line.GetOffsetToMap(curr_s, &offset_to_map);
      
          double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                    adc_frenet_ld_ * adc_frenet_ld_ /
                                    kMaxLateralAccelerations / 2.0;
      ```
    
    * 根据是否借道，计算该`S`处的path_bound,不借道时对应的bound就是上面求解的lane的宽度,左边为正值，右边为负值。
      ```C++
          double curr_left_bound_lane =
              curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                          ? curr_neighbor_lane_width
                                          : 0.0);
      
          double curr_right_bound_lane =
              -curr_lane_right_width -
              (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                   ? curr_neighbor_lane_width
                   : 0.0);
      ```
    
    * 判断此时adc的位置，根据其位置计算对应的path_bound: 取adc的l值+0.5车款+adc_buff与之前计算bound值的最大(左边界)值，最小值(左边界)
      ```C++
          if (config_.path_bounds_decider_config()
                  .is_extend_lane_bounds_to_include_adc() ||
              is_fallback_lanechange) {
            // extend path bounds to include ADC in fallback or change lane path
            // bounds.
            double curr_left_bound_adc =
                std::fmax(adc_l_to_lane_center_,
                          adc_l_to_lane_center_ + ADC_speed_buffer) +
                GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
            curr_left_bound =
                std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;
      
            double curr_right_bound_adc =
                std::fmin(adc_l_to_lane_center_,
                          adc_l_to_lane_center_ + ADC_speed_buffer) -
                GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
            curr_right_bound =
                std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
                offset_to_map;
          } else {
            curr_left_bound = curr_left_bound_lane - offset_to_map;
            curr_right_bound = curr_right_bound_lane - offset_to_map;
          }
      ```
    
    * 根据上步计算的path_bound,左右边界分别收缩0.5的车宽(左边界剪去0.5车宽，右边界加上0.5车宽)
      ```C++
          if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                            path_bound, is_left_lane_boundary,
                                            is_right_lane_boundary)) {
            path_blocked_idx = static_cast<int>(i);
          }
      ```
    
      **其中path_blocked_idx是边界中左边界小于右边界的点，被认为是障碍物点处**
    
      ```C++
      bool PathBoundsDecider::UpdatePathBoundaryWithBuffer(
          size_t idx, double left_bound, double right_bound,
          PathBound* const path_boundaries, bool is_left_lane_bound,
          bool is_right_lane_bound) {
        // substract vehicle width when bound does not come from the lane boundary
        const double default_adc_buffer_coeff = 1.0;
        double left_adc_buffer_coeff =
            (is_left_lane_bound
                 ? config_.path_bounds_decider_config().adc_buffer_coeff()
                 : default_adc_buffer_coeff);
        double right_adc_buffer_coeff =
            (is_right_lane_bound
                 ? config_.path_bounds_decider_config().adc_buffer_coeff()
                 : default_adc_buffer_coeff);
      
        // Update the right bound (l_min):
        double new_l_min =
            std::fmax(std::get<1>((*path_boundaries)[idx]),
                      right_bound + right_adc_buffer_coeff *
                                        GetBufferBetweenADCCenterAndEdge());
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
        std::get<1>((*path_boundaries)[idx]) = new_l_min;
        std::get<2>((*path_boundaries)[idx]) = new_l_max;
        return true;
      }
      ```
    
    * 根据`path_blocked_idx`对200个 path_bound剪枝
    
      ```C++
      TrimPathBounds(path_blocked_idx, path_bound);
      ```
    
      
