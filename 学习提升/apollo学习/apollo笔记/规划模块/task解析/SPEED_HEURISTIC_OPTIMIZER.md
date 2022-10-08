<center><span style="font-size:2rem;font-weight:bold;">SPEED_HEURISTIC_OPTIMIZER解析</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 总览

该task即apollo_em中的dp步骤，即根据障碍物信息，限速信息等搜索出一条cost最小的路径

totoal_cost = 与障碍物之间的距离COST+与path_终点的距离+speed_cost+acc_cost+jerk_cost

* **障碍物之间的距离cost:**

如果该s与障碍物的上边界距离超过20m，则cost为0，如果该s与障碍物下边界距离超过20m则cost为0，否则cost等于障碍物权重*（s_diff**s_diff）

* **与path_终点的距离:**

cost=(length-s)*1e2

* **speed_cost:**

det_speed = (speed - speed_limit) / speed_limit

超速：1000 * 1000 * det_speed^2

未超速：10 * 1000 * det_speed

巡航：10 * 1000 * diff_speed

* **acc_cost：**

加速度的平方*惩罚因子

* **jerk_cost：**

jerk平方

# 函数处理

## 首选i获取config

```C++
  const auto& dp_st_speed_optimizer_config =//根据是否变道，获取不同的config
      reference_line_info_->IsChangeLanePath()
          ? speed_heuristic_optimizer_config_.lane_change_speed_config()
          : speed_heuristic_optimizer_config_.default_speed_config();
```

## 之后根据st_graph_data与障碍物信息等实例化GriddedPathTimeGraph类

```C++
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);
```

GriddedPathTimeGraph构造函数,其中包含dp_st_cost的实例化构造函数

```C++
GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const std::vector<const Obstacle*>& obstacles,
    const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),
      gridded_path_time_graph_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) 
     //dp_st_cost_构造函数用来初始化cost，并将处于keep_clear区域的障碍物的start_s, end_s放入keep_clear_range_中，并对其排序
{
  total_length_t_ = st_graph_data_.total_time_by_conf();//总时间
  unit_t_ = gridded_path_time_graph_config_.unit_t();//时间分辨率1.0
  total_length_s_ = st_graph_data_.path_length();//s的总长度
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();//1.0
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();//0.1
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();//101
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
}
```

dp_st_cost的实例化构造函数

```C++
DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      st_drivable_boundary_(st_drivable_boundary),
      init_point_(init_point),
      unit_t_(config.unit_t()),
      total_s_(total_s) {
  int index = 0;
  for (const auto& obstacle : obstacles) {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }
  //遍历所有的障碍物，初始化boundary_map_[obs_id]=index

  AddToKeepClearRange(obstacles);//将处于kc区域的障碍物的start_s, end_s放入keep_clear_range_中，并对其排序

  const auto dimension_t =
      static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) +
      1;//对时间离散
  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}
```

## 之后进行dp搜索

```C++
  if (!st_graph.Search(speed_data).ok()) {//进行动态规划的搜索，返回speed_data（与A*结果类似的点集）
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
```

### Search函数

* 首先检查每个障碍物的boundary，对其检查其实点是否处于障碍物内，若是处于，则身生成速度一直为0 的speed_data结果

```C++
  for (const auto& boundary : st_graph_data_.st_boundaries()) {//遍历每个障碍物边界，检查是否有碰撞危险
    // KeepClear obstacles not considered in Dp St decision
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    //如果起点处于障碍物内则返回一系列速度点（s一直为0）
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||//如果起始点0.0处于障碍物边界内
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&//0.01
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(
                         total_length_t_ / static_cast<double>(unit_t_))) +
                     1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {//初始化网格
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));//[s,t,v=0,a=0,da=0]
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }//障碍物检查结束
```

* 之后根据config与path_data的长度初始化cost_table 

```C++
  if (!InitCostTable().ok()) {//初始化cost_table
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

```

InitCostTable函数:

**稠密s长度为10m,分辨率为0.1**

**稀疏s长度为path_length-dense_length**

**之后将每个s的index与s值对应放入spatial_distance_by_index_中**

```C++
Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {//1e-6
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;//离散时间
//以下代码即首先根据congig设置的s的个数计算预设s的长度
//如果预设长度小于总长度，在预设的维度加上剩余的维度
//如果大于，减去；；即根据unit得到s的维度
  double sparse_length_s =//稀疏s长度
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ =//稀疏的个数
      sparse_length_s > std::numeric_limits<double>::epsilon()//最小非零浮点数
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;//离散s
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;//如果没有剩余s，则s的维度等于长度/s分辨率
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));//初始化cost_table
//cost_table格式：是一个二次向量，第一维是t的个数，第二位是s的个数，对应的值为StGraphPoint
//[0  ]=[StGraphPoint][StGraphPoint]...[StGraphPoint]共N_s个
//[...]=...
//[8  ]=... 
  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {//对t进行遍历
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {//对config中的s维度进行cost_table初始化
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {//对剩余的s维度进行cost_table初始化
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = cost_table_[0];//第一个时刻的table
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);//t0时刻s的维度
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();//将每个s值放入spatial_distance_by_index_中
  }
  return Status::OK();
}
```

* 之后初始化每个s的限速信息

```C++
  if (!InitSpeedLimitLookUp().ok()) {//将每个s值的速度限制放入speed_limit_by_index_中
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

InitSpeedLimitLookUp函数

```C++
Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();//得到速度限制

  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }//将每个s值的速度限制放入speed_limit_by_index_中
  return Status::OK();
}
```

* 开始算每个格的cost,计算过程较复杂，后面详述

```C++
  if (!CalculateTotalCost().ok()) {//计算cost，obs+距离+上一点的总cost+速度+加速度+jerk
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

* 反向回溯cost最小的speed_data

```C++
  if (!RetrieveSpeedProfile(speed_data).ok()) {//通过最后的最有节点回溯速度曲线（与A*类似）
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

RetrieveSpeedProfile函数

**首先从最后一列与最后一行获取cost最小的点**

**之后以该点作为end_point，进行回溯**

**最后得到speed_data**

**最后根据s值计算每个点的速度**

```C++
Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  for (const StGraphPoint& cur_point : cost_table_.back()) {//最后一个时刻的s
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }//找到最后一列中所有s的最小cost,将其作为best_end_point

  for (const auto& row : cost_table_) {//遍历所有的列
    const StGraphPoint& cur_point = row.back();//该列的最后一个点
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }//找到最后一行中最小的cost,并与最后一个时刻的最小cost做对比，更小的将其作为best_end_point

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {//与a*算法会回溯一样，通过最优的最后一个点，一步步选择它的父节点
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}
```

# CalculateTotalCost函数

计算每个点的cost主要分为以下几步

* 首先计算该从列上每个点的total_cost
* 之后遍历该列上的每个点，根据最大加速度与最大减速度计算下一列的最大最小s值
* 之后继续第一个步骤，计算列上每个点的cost

```C++
for (size_t c = 0; c < cost_table_.size(); ++c) {//列
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {//初始为1
      std::vector<std::future<void>> results;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {//根据前一时刻计算下一时刻的s范围
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {//false
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
          CalculateCostAt(msg);
        }
      }
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {//false
        for (auto& result : results) {
          result.get();
        }
      }
    }

    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);//根据当前点的速度与s值以及单位时间计算下一时刻以最大加速度和最大减速度计算的最大和最小s值
        //即下一时刻的s值范围
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
```

## GetRowRange函数

```C++
void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();//根据最大加速度计算单位时间后的s值
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }//通过当前点速度与时间计算下一个时间的最大s值

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();//同理，以最大减速度计算单位时间t后的s值
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }//最小s值
}
```

## CalculateCostAt函数

### 首先计算与障碍物之间距离的cost

```C++
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));//计算该点处障碍物的cost（与障碍物之间的距离）
  //遍历所有障碍物，若计算cost=1.0*1e4*dx^2
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }
```

GetObstacleCost函数

* 首先判断该点的s是否处于st_bound_decider中计算的[s_lower,s_upper]范围中，若是处于则继续，反之返回无穷大

```C++
  if (FLAGS_use_st_drivable_boundary) {//true
    // TODO(Jiancheng): move to configs
    static constexpr double boundary_resolution = 0.1;
    int index = static_cast<int>(t / boundary_resolution);
    const double lower_bound =
        st_drivable_boundary_.st_boundary(index).s_lower();
    const double upper_bound =
        st_drivable_boundary_.st_boundary(index).s_upper();

    if (s > upper_bound || s < lower_bound) {
      return kInf;//无穷
    }
  }
```

* 之后遍历所有的障碍物

跳过虚拟障碍物

跳过纵向决策为stop的障碍物

跳过在该时间t之前与之后的障碍物

```C++
    if (obstacle->IsVirtual()) {
      continue;
    }

    // Stop obstacles are assumed to have a safety margin when mapping them out,
    // so repelling force in dp st is not needed as it is designed to have adc
    // stop right at the stop distance we design in prior mapping process
    if (obstacle->LongitudinalDecision().has_stop()) {//纵向停车
      continue;
    }

    auto boundary = obstacle->path_st_boundary();

    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {//200
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (boundary.IsPointInBoundary(st_graph_point.point())) {//该点处于障碍物内
      return kInf;//返回无穷
    }
```

计算该障碍物在该时间t上的上下边界

```C++
    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {//刚开始
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);//计算障碍物的在该时刻t的上下边界值
    } else {//计算过了
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
```

如果该s与障碍物的上边界距离超过20m，则cost为0，如果该s与障碍物下边界距离超过20m则cost为0，否则cost等于障碍物权重*（s_diff**s_diff）

```C++
    if (s < s_lower) {//如果这个s的值小于障碍物的下边界
      const double follow_distance_s = config_.safe_distance();//20
      if (s + follow_distance_s < s_lower) {//若是当前s值加上20仍小于障碍物的下边界，则不计算cost
        continue;
      } else {
        auto s_diff = follow_distance_s - s_lower + s;//距离差值
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;//1.0*1e4*dx^2
                //obstacle_weight=1.0,default_obstacle_cost=1e4
      }
    } else if (s > s_upper) {//超出上边界
      const double overtake_distance_s =
          StGapEstimator::EstimateSafeOvertakingGap();//20.0
      if (s > s_upper + overtake_distance_s) {  // or calculated from velocity
        continue;
      } else {
        auto s_diff = overtake_distance_s + s_upper - s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }//与上同理
```

### 之后计算与path_终点距离的cost

```C++
 cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));//计算与目标的距离的cost
  //cost=(length-s)*1e2
```

### 之后计算每个点的最优前一个点与速度cost，加速度cost，jerk_cost

#### 如果t==0,即第一列

此时只有一个点即起始点，因此没有前一个点，最优速度为起始点速度，total_cost 为0

```C++
  const auto& cost_init = cost_table_[0][0];//StGraphPoint
  if (c == 0) {//第一列，第一个时间t，起始点的cost为0，且限速为起始速度
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }
```

#### 如果t==1,即第二列

此时每个点的前一个点都是起始点

* 首先计算该点的加速度，若是加速度超出动力学范围，返回

```C++
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;
    if (acc < max_deceleration_ || acc > max_acceleration_) {//
      return;
    }
```

* 如果该点的速度小于0且s值大于0.8,则返回

```C++
    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&//速度小于0
        cost_cr.point().s() > min_s_consider_speed) {//该点s大于1.75m
      return;
    }
```

* 计算速度cost，acc_cost,jerk_cost,并加入总cost，设置该点速度与前一个点

```C++
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;
```

CalculateEdgeCostForSecondCol函数

```C++
double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,//速度差值/时间
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +//加速度的平方*惩罚因子
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);//jerk平方
}
```

GetSpeedCost函数

首先计算该点的速度

```C++
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return kInf;
  }
```

如果车已经停下，且处于kc区域，则计算cost

```C++
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }
```

计算限速cost,巡航速度cost

```C++
  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;//1000*1000*det_speed^2
  } else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;//10*1000*det_speed
  }

  if (FLAGS_enable_dp_reference_speed) {
    double diff_speed = speed - cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;//10*1000*diff_speed
  }
```

GetAccelCostByTwoPoints函数

```C++
double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();//2
    double max_dec = config_.max_deceleration();//-4
    double accel_penalty = config_.accel_penalty();//1
    double decel_penalty = config_.decel_penalty();//1

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}
```

GetJerkCostByTwoPoints函数

```C++
double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}
```

* 设置total_cost

```C++
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
```

#### 如果t==2

会计算一个前一列的父节点范围，遍历父节点，选择cost最小的点作为其父节点

```C++
 if (c == 2) {//第二列
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
           pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;
  }
```

#### T==其他

操作以t==2相同，只是在计算speed_cost有所区别

```C++
 for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                              curr_a * unit_t_);
    }
  }
```

