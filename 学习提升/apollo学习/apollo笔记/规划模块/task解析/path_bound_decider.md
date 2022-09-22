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
      ```
  
      
