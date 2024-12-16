// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_path_planner/utils/pull_out/single_arc_pull_out.hpp"

#include "behavior_path_planner/utils/pull_out/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

using lanelet::utils::getArcCoordinates;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
using pull_out_utils::combineReferencePath;
using pull_out_utils::getPullOutLanes;

SingleArcPullOut::SingleArcPullOut(rclcpp::Node & node, const PullOutParameters & parameters)
: PullOutPlannerBase{node, parameters}
{
  planner_.single_arc_pull_out_velocity_ = parameters_.single_arc_pull_out_velocity;
  planner_.single_arc_path_interval_ = parameters_.single_arc_path_interval;
  planner_.single_arc_lane_departure_margin_ = parameters_.single_arc_lane_departure_margin;
  planner_.single_arc_max_steer_angle_ = parameters_.single_arc_max_steer_angle;
}

boost::optional<PullOutPath> SingleArcPullOut::plan(Pose start_pose, Pose goal_pose)
{
  PullOutPath output;

  // combine road lane and shoulder lane
  const auto road_lanes = utils::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = getPullOutLanes(planner_data_);
  auto lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());
  for(const auto &lane : lanes)
  {
    std::cerr<<"Road lanes:"<<lane.id()<<std::endl;
  }

  planner_.setTurningRadius(
      planner_data_->parameters, parameters_.single_arc_max_steer_angle);
  planner_.setPlannerData(planner_data_);
  const bool found_valid_path =
    planner_.planPullOutSingleArc(start_pose, goal_pose, road_lanes, shoulder_lanes);
  if (!found_valid_path) {
    std::cerr<<"SingleArcPullOut::plan failed"<<std::endl;
    return {};
  }

  // collision check with objects in shoulder lanes
  const auto arc_path = planner_.getArcPath();
  const auto [shoulder_lane_objects, others] =
    utils::separateObjectsByLanelets(*(planner_data_->dynamic_object), shoulder_lanes);
  if (utils::checkCollisionBetweenPathFootprintsAndObjects(
        vehicle_footprint_, arc_path, shoulder_lane_objects, parameters_.collision_check_margin)) {
    return {};
  }

  output.partial_paths = planner_.getPaths();
  output.start_pose = planner_.getArcPaths().at(0).points.front().point.pose;
  output.end_pose = planner_.getArcPaths().at(0).points.back().point.pose;

  return output;
}
}  // namespace behavior_path_planner
