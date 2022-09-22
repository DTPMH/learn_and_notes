/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(const TaskConfig& config)
    : SpeedOptimizer(config) {
  ACHECK(config.has_speed_heuristic_optimizer_config());
  speed_heuristic_optimizer_config_ = config.speed_heuristic_optimizer_config();
}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  const auto& dp_st_speed_optimizer_config =//根据是否变道，获取不同的config
      reference_line_info_->IsChangeLanePath()
          ? speed_heuristic_optimizer_config_.lane_change_speed_config()
          : speed_heuristic_optimizer_config_.default_speed_config();
  
  //构建st_graph，初始化速度规划的网格图
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);

  if (!st_graph.Search(speed_data).ok()) {//进行动态规划的搜索，返回speed_data（与A*结果类似的点集）
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}

Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {//均为参考线数据
  init_point_ = init_point;

  if (path_data.discretized_path().empty()) {//判断路径是否可用
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!SearchPathTimeGraph(speed_data)) {//进行动态规划
    const std::string msg = absl::StrCat(
        Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  RecordDebugInfo(//打印调试信息
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
