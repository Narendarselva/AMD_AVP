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

#include "behavior_path_planner/utils/single_arc/single_arc.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <string>
#include <utility>
#include <vector>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using lanelet::utils::getArcCoordinates;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::deg2rad;
using tier4_autoware_utils::inverseTransformPoint;
using tier4_autoware_utils::inverseTransformPose;
using tier4_autoware_utils::normalizeRadian;
using tier4_autoware_utils::toMsg;
using tier4_autoware_utils::transformPose;

namespace behavior_path_planner
{
void SingleArc::incrementPathIndex()
{
  current_path_idx_ = std::min(current_path_idx_ + 1, paths_.size() - 1);
}

PathWithLaneId SingleArc::getPathByIdx(size_t const idx) const
{
  if (paths_.empty() || paths_.size() <= idx) {
    return PathWithLaneId{};
  }

  return paths_.at(idx);
}

PathWithLaneId SingleArc::getCurrentPath() const
{
  return paths_.at(current_path_idx_);
}

PathWithLaneId SingleArc::getFullPath() const
{
  PathWithLaneId path{};
  for (const auto & partial_path : paths_) {
    path.points.insert(path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  PathWithLaneId filtered_path = path;
  filtered_path.points = motion_utils::removeOverlapPoints(filtered_path.points);
  return filtered_path;
}

PathWithLaneId SingleArc::getArcPath() const
{
  PathWithLaneId path{};
  for (const auto & arc_path : arc_paths_) {
    path.points.insert(path.points.end(), arc_path.points.begin(), arc_path.points.end());
  }
  return path;
}

bool SingleArc::isParking() const
{
  return current_path_idx_ > 0;
}

void SingleArc::setVelocityToArcPaths(
  std::vector<PathWithLaneId> & arc_paths, const double velocity, const bool set_stop_end)
{
  for (auto & path : arc_paths) {
    for (size_t i = 0; i < path.points.size(); i++) {
      if (i == path.points.size() - 1 && set_stop_end) {
        // stop point at the end of the path
        path.points.at(i).point.longitudinal_velocity_mps = 0.0;
      } else {
        path.points.at(i).point.longitudinal_velocity_mps = velocity;
      }
    }
  }
}

void SingleArc::clearPaths()
{
  current_path_idx_ = 0;
  arc_paths_.clear();
  paths_.clear();
}

bool SingleArc::planPullOutSingleArc(
  const Pose & start_pose, const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes)
{
  int sign_distance;
  bool isLeftSide = false;
  constexpr bool is_forward = false;         // parking backward means pull_out forward
  constexpr double max_offset = 10.0;
  constexpr double offset_interval = 0.01;
  //max steering set to 30deg
  const auto common_params = planner_data_->parameters;

  //Find whether the shoulder lane is right or left of center lane 
  const auto start_pose_arc_coordinates = lanelet::utils::getArcCoordinates(road_lanes, start_pose);
  if(start_pose_arc_coordinates.distance > 0)
  {
    isLeftSide = true;
    sign_distance = 1; //left of center lane
    std::cerr<<"Left of center lane"<<std::endl;
  }
  else
  {
    sign_distance = -1; //Right of center lane
    std::cerr<<"Right of center lane"<<std::endl;
  }

  for (double start_pose_offset = 0; start_pose_offset < max_offset;start_pose_offset += offset_interval)
  {
    const Pose modified_start_pose = calcOffsetPose(start_pose,start_pose_offset,0,0);

    auto arc_paths = planOneTrialSingleArc(
        modified_start_pose , R_E_min_, road_lanes, shoulder_lanes, is_forward, start_pose_offset, 0.0,
        single_arc_lane_departure_margin_ , single_arc_path_interval_ ,isLeftSide);
    if (arc_paths.empty()) {
      // not found path
      continue;
    }

    // Not use shoulder lanes.
    auto check_pose = arc_paths.back().points.back().point.pose;
    const auto arc_coordinates = lanelet::utils::getArcCoordinates(road_lanes, check_pose);
    if((sign_distance * arc_coordinates.distance) > 0.0)
    {
      continue;
    }

    //To check yaw diff between arc end pose and beginning of the road center
    //lane
    for(double end_yaw = 0.0; end_yaw < 1.57; end_yaw+=0.0174533)
    {
      auto check_pose = arc_paths.back().points.back().point.pose;

      //Update the current ego pose to the path
      if(start_pose_offset != 0)
      {
        auto path_point = arc_paths.front().points.back();
        path_point.point.pose = start_pose;
        arc_paths.back().points.insert(arc_paths.back().points.begin(),path_point);
      }

      // get road center line path from pull_out end to goal, and combine after the second arc path
      const double s_start = getArcCoordinates(road_lanes, check_pose).length;
      const double s_goal = getArcCoordinates(road_lanes, goal_pose).length;
      const double road_lanes_length = std::accumulate(
          road_lanes.begin(), road_lanes.end(), 0.0, [](const double sum, const auto & lane) {
          return sum + lanelet::utils::getLaneletLength2d(lane);
          });
      const bool goal_is_behind = s_goal < s_start;
      const double s_end = goal_is_behind ? road_lanes_length : s_goal;
      PathWithLaneId road_center_line_path =
        planner_data_->route_handler->getCenterLinePath(road_lanes, s_start, s_end, true);

      // check the continuity of straight path and arc path
      const Pose & road_path_first_pose = road_center_line_path.points.front().point.pose;
      const Pose & arc_path_last_pose = arc_paths.back().points.back().point.pose;
      const double yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(
            tf2::getYaw(road_path_first_pose.orientation) - tf2::getYaw(arc_path_last_pose.orientation)));

      //if yaw diff is > 10deg rerun the OnTrail with updated end_yaw
      if(yaw_diff > 0.174533)//10 deg
      {
        arc_paths = planOneTrialSingleArc(
            modified_start_pose , R_E_min_, road_lanes, shoulder_lanes, is_forward, start_pose_offset, end_yaw,
            single_arc_lane_departure_margin_ , single_arc_path_interval_ ,isLeftSide);
        continue;
      }

      // combine the road center line path with the second arc path
      auto paths = arc_paths;
      paths.back().points.insert(
          paths.back().points.end(),
          road_center_line_path.points.begin() + 1,  // to avoid overlapped point
          road_center_line_path.points.end());
      paths.back().points = motion_utils::removeOverlapPoints(paths.back().points);
      setVelocityToArcPaths(paths, single_arc_pull_out_velocity_, true);

      //Set zero velocity at the arc end point this is to fix issue when the end
      //of arc is in a curve path when it switches from summon to lanedriving
      //path
      paths.back().points.at(arc_paths.back().points.size()-1).point.longitudinal_velocity_mps = 0.0;

      arc_paths_ = arc_paths;
      paths_ = paths;
      std::cerr<<"Arc size: "<<arc_paths_.back().points.size()<<std::endl;
      std::cerr<<"Road centerline size: "<<road_center_line_path.points.size()<<std::endl;
      std::cerr<<"Path size: "<<paths_.back().points.size()<<std::endl;
      std::cerr<<"Final yaw diff: "<<yaw_diff<<std::endl;
      return true;
    }
  }
  std::cerr<<"SingleArc::planPullOutSingleArc planOneTrail failed"<<std::endl;
  return false;
}

PathWithLaneId SingleArc::generateStraightPath(const Pose & start_pose)
{
  // get straight path before parking.
  const auto current_lanes = utils::getExtendedCurrentLanes(planner_data_);
  const auto start_arc_position = lanelet::utils::getArcCoordinates(current_lanes, start_pose);

  const Pose current_pose = planner_data_->self_odometry->pose.pose;
  const auto current_arc_position = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  auto path = planner_data_->route_handler->getCenterLinePath(
    current_lanes, current_arc_position.length, start_arc_position.length, true);
  path.header = planner_data_->route_handler->getRouteHeader();
  if (!path.points.empty()) {
    path.points.back().point.longitudinal_velocity_mps = 0;
  }

  return path;
}

std::vector<PathWithLaneId> SingleArc::planOneTrialSingleArc(
  const Pose & start_pose, const double R_E_min,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const bool is_forward, const double start_pose_offset, const double end_yaw,const double lane_departure_margin,
  const double arc_path_interval, bool isLeftSide)
{
  (void) start_pose_offset;
  (void) lane_departure_margin;
  clearPaths();
  PathWithLaneId path_turn;

  const auto common_params = planner_data_->parameters;
  //const double self_yaw = tf2::getYaw(start_pose.orientation);

  // combine road and shoulder lanes
  // cut the road lanes up to start_pose to prevent unintended processing for overlapped lane
  lanelet::ConstLanelets lanes{};
  tier4_autoware_utils::Point2d start_point2d(start_pose.position.x, start_pose.position.y);
  for (const auto & lane : road_lanes) {
    if (boost::geometry::within(start_point2d, lane.polygon2d().basicPolygon())) {
      lanes.push_back(lane);
      break;
    }
    lanes.push_back(lane);
  }
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  if(isLeftSide)
  {
    // Generate arc path(left turn)
    const Pose Cl = calcOffsetPose(start_pose,0,R_E_min,0);
    path_turn = generateArcPath(
        Cl, R_E_min, -M_PI_2,-end_yaw, arc_path_interval, isLeftSide,
        !is_forward);
  }
  else
  {
    // Generate arc path(Right turn)
    const Pose Cr = calcOffsetPose(start_pose,0,-R_E_min,0);
    path_turn = generateArcPath(
        Cr, R_E_min, M_PI_2,end_yaw, arc_path_interval, isLeftSide,
        !is_forward);
  }

  path_turn.header = planner_data_->route_handler->getRouteHeader();

  auto setLaneIds = [lanes](PathPointWithLaneId & p) {
    for (const auto & lane : lanes) {
      p.lane_ids.push_back(lane.id());
    }
  };
  auto setLaneIdsToPath = [setLaneIds](PathWithLaneId & path) {
    for (auto & p : path.points) {
      setLaneIds(p);
    }
  };

  setLaneIdsToPath(path_turn);
  // generate arc path vector
  paths_.push_back(path_turn);

  return paths_;
}

PathWithLaneId SingleArc::generateArcPath(
  const Pose & center, const double radius, const double start_yaw, double end_yaw,
  const double arc_path_interval,
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  PathWithLaneId path;
  const double yaw_interval = arc_path_interval / radius;
  double yaw = start_yaw;
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw += yaw_interval;
    }
  } else {  // right_turn
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw -= yaw_interval;
    }
  }

  // insert the last point exactly
  const auto p = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  constexpr double min_dist = 0.01;
  if (path.points.empty() || calcDistance2d(path.points.back(), p) > min_dist) {
    path.points.push_back(p);
  }

  return path;
}

PathPointWithLaneId SingleArc::generateArcPathPoint(
  const Pose & center, const double radius, const double yaw, const bool is_left_turn,
  const bool is_forward)
{
  // get pose in center_pose coords
  Pose pose_center_coords;
  pose_center_coords.position.x = radius * std::cos(yaw);
  pose_center_coords.position.y = radius * std::sin(yaw);

  // set orientation
  tf2::Quaternion quat;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quat.setRPY(0, 0, normalizeRadian(yaw - M_PI_2));
  } else {
    quat.setRPY(0, 0, normalizeRadian(yaw + M_PI_2));
  }
  pose_center_coords.orientation = tf2::toMsg(quat);

  // get pose in map coords
  PathPointWithLaneId p{};
  p.point.pose = transformPose(pose_center_coords, center);

  return p;
}

void SingleArc::setTurningRadius(
  const BehaviorPathPlannerParameters & common_params, const double max_steer_angle)
{
  R_E_min_ = common_params.wheel_base / std::tan(max_steer_angle);
  R_Bl_min_ = std::hypot(
    R_E_min_ + common_params.wheel_tread / 2 + common_params.left_over_hang,
    common_params.wheel_base + common_params.front_overhang);
}

}  // namespace behavior_path_planner
