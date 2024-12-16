// Copyright 2020 Tier IV, Inc.
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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/polygon_to_costmap.hpp"

#include <string>
#include <vector>

void PolygonToCostmap::initGridmapParam(const grid_map::GridMap & gridmap)
{
  grid_length_x_ = gridmap.getLength().x();
  grid_length_y_ = gridmap.getLength().y();
  grid_resolution_ = gridmap.getResolution();
  grid_position_x_ = gridmap.getPosition().x();
  grid_position_y_ = gridmap.getPosition().y();
}

bool PolygonToCostmap::isValidInd(const grid_map::Index & grid_ind)
{
  bool is_valid = false;
  int x_grid_ind = grid_ind.x();
  int y_grid_ind = grid_ind.y();
  if (
    x_grid_ind >= 0 && x_grid_ind < std::ceil(grid_length_x_ * (1 / grid_resolution_)) &&
    y_grid_ind >= 0 && y_grid_ind < std::ceil(grid_length_y_ * (1 / grid_resolution_))) {
    is_valid = true;
  }
  return is_valid;
}

grid_map::Index PolygonToCostmap::fetchGridIndexFromPoint(double x,double y)
{
  int mapped_x_ind = 0;
  int mapped_y_ind = 0;
  // calculate out_grid_map position
  const double origin_x_offset = grid_length_x_ / 2.0  - grid_position_x_;
  const double origin_y_offset = grid_length_y_ / 2.0  - grid_position_y_;
  // coordinate conversion for making index. Set bottom left to the origin of coordinate (0, 0) in
  // gridmap area
  double mapped_x = (grid_length_x_ - origin_x_offset - x) / grid_resolution_;
  double mapped_y = (grid_length_y_ - origin_y_offset - y) / grid_resolution_;

  //NAREN -fix to align grid cell with detected polygon
  if(mapped_x < 0)
  {
    mapped_x_ind = std::ceil(mapped_x);
  }
  else
  {
    mapped_x_ind = std::floor(mapped_x);
  }

  if(mapped_y < 0)
  {
    mapped_y_ind = std::ceil(mapped_y);
  }
  else
  {
    mapped_y_ind = std::floor(mapped_y);
  }

  grid_map::Index index(mapped_x_ind, mapped_y_ind);
  return index;
}

/*std::vector<std::vector<std::vector<double>>> PolygonToCostmap::assignPoints2GridCell(
   const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & in_freespace_polygon)
{
  double y_cell_size = std::ceil(grid_length_y_ * (1 / grid_resolution_));
  double x_cell_size = std::ceil(grid_length_x_ * (1 / grid_resolution_));
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);
  geometry_msgs::msg::Point32 point1 = in_freespace_polygon->polygon.points.front();

  for (auto it = in_freespace_polygon->polygon.points.begin();
		  it != in_freespace_polygon->polygon.points.end(); ++it)
  {
    if( it != in_freespace_polygon->polygon.points.begin())
    {
      geometry_msgs::msg::Point32 point2 = *it;
      addPoints(point2,point1,vec_x_y_z);
      point1 = *it;
    }
  }
  addPoints(in_freespace_polygon->polygon.points.front(),point1,vec_x_y_z);
  return vec_x_y_z;
}*/

void PolygonToCostmap::addPoints(geometry_msgs::msg::Point32 point1,geometry_msgs::msg::Point32 point2,
  const double grid_min_value, const double grid_max_value,const grid_map::GridMap & gridmap,
  grid_map::Matrix gridmap_data,nav_msgs::msg::OccupancyGrid & out_occupancy_grid)
{
   grid_map::Index grid_ind;
   double dx = point2.x -point1.x;
   double dy = point2.y -point1.y;
   double distance = sqrt(dx*dx + dy*dy);
   int numPoints = std::ceil(distance * (1 / grid_resolution_));

   size_t nCells = gridmap.getSize().prod();
   const float cellMin = 0;
   const float cellMax = 100;
   const float cellRange = cellMax - cellMin;

   dx /= distance;
   dy /= distance;
 
   for (int i = 0; i <= numPoints  ; i++)
   {
     double x = point1.x + ((i * grid_resolution_) * dx);
     double y = point1.y + ((i * grid_resolution_) * dy);

     grid_ind = fetchGridIndexFromPoint(x,y);
     if (isValidInd(grid_ind)) {
       int x_ind = grid_ind.x();
       int y_ind = grid_ind.y();
       gridmap_data(x_ind, y_ind) = grid_max_value;

       grid_map::Index grid_index(x_ind,y_ind);
       float value = (grid_max_value - grid_min_value) / (grid_max_value - grid_min_value);
       value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
       size_t index = grid_map::getLinearIndexFromIndex(grid_index, gridmap.getSize(), false);
       out_occupancy_grid.data[nCells - index - 1] = value;
     }
   }

   grid_ind = fetchGridIndexFromPoint(point2.x,point2.y);
   if (isValidInd(grid_ind)) {
       int x_ind = grid_ind.x();
       int y_ind = grid_ind.y();
       gridmap_data(x_ind, y_ind) = grid_max_value;

       grid_map::Index grid_index(x_ind,y_ind);
       float value = (grid_max_value - grid_min_value) / (grid_max_value - grid_min_value);
       value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
       size_t index = grid_map::getLinearIndexFromIndex(grid_index, gridmap.getSize(), false);
       out_occupancy_grid.data[nCells - index - 1] = value;
   }

}

grid_map::Matrix PolygonToCostmap::calculateCostmap(
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,
  const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & in_freespace_polygon,
  nav_msgs::msg::OccupancyGrid & out_occupancy_grid)
{
  // NAREN
  // Modified the whole logic here.
  // Instead of populating an 3 dimension vector and then looping that to
  // populate the grid and occupancy , now we are directly populating it from
  // the input polygon points
  grid_map::Matrix gridmap_data = gridmap[gridmap_layer_name];

  if(gridmap_layer_name == "polygon")
  {
    geometry_msgs::msg::Point32 point1 = in_freespace_polygon->polygon.points.front();
    for (auto it = in_freespace_polygon->polygon.points.begin();
        it != in_freespace_polygon->polygon.points.end(); ++it)
    {
      if( it != in_freespace_polygon->polygon.points.begin())
      {
        geometry_msgs::msg::Point32 point2 = *it;
        addPoints(point2,point1,grid_min_value,grid_max_value,gridmap,gridmap_data,out_occupancy_grid);
        point1 = *it;
      }
    }
    addPoints(in_freespace_polygon->polygon.points.front(),point1,grid_min_value,grid_max_value,gridmap,gridmap_data,out_occupancy_grid);
  }
  else if(gridmap_layer_name == "marker")
  {
    //For Freespace from ultrasonic marker array
    //the wedges are converted into polygon points
    geometry_msgs::msg::Point32 point1 = in_freespace_polygon->polygon.points.front();
    for (auto it = in_freespace_polygon->polygon.points.begin();
        it != in_freespace_polygon->polygon.points.end();++it)
    {
      if( it != in_freespace_polygon->polygon.points.begin())
      {
        //This is check the separator between individual sensor o/p
        if((it->x == FLT_MAX) && (it->y == FLT_MAX))
        {
          ++it;
          if(it != in_freespace_polygon->polygon.points.end())
          {
            //Not the final point
            point1 = *it;
            continue;
          }
          else
          {
            //Final point
            break;
          }
        }
        geometry_msgs::msg::Point32 point2 = *it;
        addPoints(point2,point1,grid_min_value,grid_max_value,gridmap,gridmap_data,out_occupancy_grid);
        point1 = *it;
      }
    }
  }

  return gridmap_data;
}

grid_map::Matrix PolygonToCostmap::makeCostmapFromPolygon(
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,  const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & in_freespace_polygon,nav_msgs::msg::OccupancyGrid & occupancyGrid)
{
  initGridmapParam(gridmap);
  grid_map::Matrix costmap = calculateCostmap(grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name,in_freespace_polygon,occupancyGrid);
  return costmap;
}
