/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/occupancy_grid.h"
#include <algorithm>
#include <vector>
#include "cartographer_ros/map_writer.h"
#include "cartographer/transform/transform.h"

#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"

namespace {

double lidar_location_x;
double lidar_location_y;
//double box_max_x;
double box_min_x;
//double box_max_y;
double box_min_y;

double x,y,z,w;

double yaw;

Eigen::AlignedBox2f ComputeMapBoundingBox2D(
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes) {
  Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());
  //int tag = 0;
  for (const auto& trajectory_nodes : all_trajectory_nodes) {
    for (const auto& node : trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      const auto& data = *node.constant_data;
      ::cartographer::sensor::RangeData range_data;
      range_data = ::cartographer::sensor::TransformRangeData(
          Decompress(data.range_data), node.pose.cast<float>());
      bounding_box.extend(range_data.origin.head<2>());

      lidar_location_x = range_data.origin.x();
      lidar_location_y = range_data.origin.y();

      w = node.pose.rotation().w();
      x = node.pose.rotation().x();
      y = node.pose.rotation().y();
      z = node.pose.rotation().z();

      yaw = ::cartographer::transform::GetYaw(node.pose.rotation());

      //double angle_mnf  = yaw / 3.14159 * 180;

      //LOG(INFO) << "angle_mnf = " << angle_mnf << "end";


      //if(tag == 0)
      //{
        //box_max_x = range_data.returns[0].x();
        //box_min_x = range_data.returns[0].x();
        //box_max_y = range_data.returns[0].y();
        //box_min_y = range_data.returns[0].y();
      //}
      //tag++;

      for (const Eigen::Vector3f& hit : range_data.returns) {
        bounding_box.extend(hit.head<2>());

        //if(hit.x() > box_max_x)
        //     box_max_x = hit.x();
        //if(hit.x() < box_min_x)
        //     box_min_x = hit.x();
        //if(hit.y() > box_max_y)
        //     box_max_y = hit.y();
        //if(hit.y() < box_min_y)
        //     box_min_y = hit.y();
      }
      for (const Eigen::Vector3f& miss : range_data.misses) {
        bounding_box.extend(miss.head<2>());

        //if(miss.x() > box_max_x)
        //     box_max_x = miss.x();
        //if(miss.x() < box_min_x)
        //     box_min_x = miss.x();
        //if(miss.y() > box_max_y)
        //     box_max_y = miss.y();
        //if(miss.y() < box_min_y)
        //     box_min_y = miss.y();
      }
    }
  }
  return bounding_box;
}

}  // namespace

namespace cartographer_ros {

void BuildOccupancyGrid2D(
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    ::nav_msgs::OccupancyGrid* const occupancy_grid) {
  namespace carto = ::cartographer;

  const carto::mapping_2d::MapLimits map_limits =
      ComputeMapLimits(submaps_options.resolution(), all_trajectory_nodes);
      
  carto::mapping_2d::ProbabilityGrid probability_grid(map_limits);
  carto::mapping_2d::RangeDataInserter range_data_inserter(
      submaps_options.range_data_inserter_options());
  carto::common::Time latest_time = carto::common::Time::min();
  for (const auto& trajectory_nodes : all_trajectory_nodes) {
    for (const auto& node : trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      latest_time = std::max(latest_time, node.time());
      range_data_inserter.Insert(carto::sensor::TransformRangeData(
                                     Decompress(node.constant_data->range_data),
                                     node.pose.cast<float>()),
                                 &probability_grid);
    }
  }
  CHECK(latest_time != carto::common::Time::min());
  occupancy_grid->header.stamp = ToRos(latest_time);
  occupancy_grid->header.frame_id = map_frame;
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;

  Eigen::Array2i offset;
  carto::mapping_2d::CellLimits cell_limits;
  //probability_grid.ComputeCroppedLimits(&offset, &cell_limits);
  offset = Eigen::Array2i::Zero();
  cell_limits = map_limits.cell_limits();

  const double resolution = probability_grid.limits().resolution();

  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = cell_limits.num_y_cells;
  occupancy_grid->info.height = cell_limits.num_x_cells;
/*
  occupancy_grid->info.origin.position.x =
      probability_grid.limits().max().x() -
      (offset.y() + cell_limits.num_y_cells) * resolution;
  occupancy_grid->info.origin.position.y =
      probability_grid.limits().max().y() -
      (offset.x() + cell_limits.num_x_cells) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = offset.y();
  occupancy_grid->info.origin.orientation.x = probability_grid.limits().max().x();
  occupancy_grid->info.origin.orientation.y = probability_grid.limits().max().y();
  occupancy_grid->info.origin.orientation.z = offset.x();
*/

  occupancy_grid->info.origin.position.x =::cartographer::common::RoundToInt((lidar_location_x - box_min_x) / resolution);
  occupancy_grid->info.origin.position.y = cell_limits.num_x_cells - ::cartographer::common::RoundToInt((lidar_location_y - box_min_y) / resolution);
  occupancy_grid->info.origin.position.z = yaw;
  occupancy_grid->info.origin.orientation.w = w;
  occupancy_grid->info.origin.orientation.x = x;
  occupancy_grid->info.origin.orientation.y = y;
  occupancy_grid->info.origin.orientation.z = z;
  occupancy_grid->data.resize(cell_limits.num_x_cells * cell_limits.num_y_cells,
                              -1);

  for (const Eigen::Array2i& xy_index :
       carto::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      const int value = carto::common::RoundToInt(
          (probability_grid.GetProbability(xy_index + offset) -
           carto::mapping::kMinProbability) *
          100. /
          (carto::mapping::kMaxProbability - carto::mapping::kMinProbability));
      CHECK_LE(0, value);
      CHECK_GE(100, value);
      occupancy_grid->data[(cell_limits.num_x_cells - xy_index.x()) *
                               cell_limits.num_y_cells -
                           xy_index.y() - 1] = value;
    }
  }
  cartographer_ros::WriteOccupancyGridToPgmAndYaml(*occupancy_grid,"/home/zkma/OUTMAP/outmap");
}

::cartographer::mapping_2d::MapLimits ComputeMapLimits(
    const double resolution,
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes) {
  Eigen::AlignedBox2f bounding_box =
      ComputeMapBoundingBox2D(all_trajectory_nodes);
  // Add some padding to ensure all rays are still contained in the map after
  // discretization.
  const float kPadding = 3.f * resolution;

  bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
  bounding_box.max() += kPadding * Eigen::Vector2f::Ones();
  box_min_x = bounding_box.min().x();
  box_min_y = bounding_box.min().y();

  const Eigen::Vector2d pixel_sizes =
      bounding_box.sizes().cast<double>() / resolution;
  return ::cartographer::mapping_2d::MapLimits(
      resolution, bounding_box.max().cast<double>(),
      ::cartographer::mapping_2d::CellLimits(
          ::cartographer::common::RoundToInt(pixel_sizes.y()),
          ::cartographer::common::RoundToInt(pixel_sizes.x())));
}

}  // namespace cartographer_ros
