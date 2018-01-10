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

#include <sstream> 
#include <string> 

#include <math.h>

#include <QUdpSocket>

#define PI 3.14159262

using namespace std; 

using namespace cv;

namespace {

double lidar_location_x;
double lidar_location_y;

double box_min_x;

double box_min_y;

double x,y,z,w;

double yaw;


double NormAngle(double angle)
{
  while(angle > PI)
    angle = angle - 2*PI;
  while(angle < -PI)
    angle = angle + 2*PI; 

  return angle;
}

//----------------------> angle = 0
void CalAngleAndDist(int shipRow, int shipCol, int obstacleRow, int obstacleCol, double& angle, double& dist)
{
  double vectorY = -(obstacleRow - shipRow);
  double vectorX = obstacleCol - shipCol;
  dist = sqrt(vectorY*vectorY + vectorX*vectorX);
  angle = atan(vectorY/vectorX);
  if(vectorY < 0)
    angle = angle - PI;
}

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

      for (const Eigen::Vector3f& hit : range_data.returns) {
        bounding_box.extend(hit.head<2>());
      }
      for (const Eigen::Vector3f& miss : range_data.misses) {
        bounding_box.extend(miss.head<2>());

      }
    }
  }
  return bounding_box;
}

}  // namespace

int map_num = 0;

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

  occupancy_grid->info.origin.position.x =::cartographer::common::RoundToInt((lidar_location_x - box_min_x) / resolution);
  occupancy_grid->info.origin.position.y = cell_limits.num_x_cells - ::cartographer::common::RoundToInt((lidar_location_y - box_min_y) / resolution);
  occupancy_grid->info.origin.position.z = yaw;
  occupancy_grid->info.origin.orientation.w = w;
  occupancy_grid->info.origin.orientation.x = x;
  occupancy_grid->info.origin.orientation.y = y;
  occupancy_grid->info.origin.orientation.z = z;
  occupancy_grid->data.resize(cell_limits.num_x_cells * cell_limits.num_y_cells,
                              -1);

  Mat mat_occupancy_grid = Mat::zeros(cell_limits.num_x_cells,cell_limits.num_y_cells,CV_8UC1);

  for (const Eigen::Array2i& xy_index :
       carto::mapping_2d::XYIndexRangeIterator(cell_limits)) {

    
    if (probability_grid.IsKnown(xy_index + offset)) {
      const int value = carto::common::RoundToInt(
          (probability_grid.GetProbability(xy_index + offset) -
           carto::mapping::kMinProbability) *
          255. /
          (carto::mapping::kMaxProbability - carto::mapping::kMinProbability));
      CHECK_LE(0, value);
      CHECK_GE(255, value);
      if(value > 130)
      {
        mat_occupancy_grid.at<uchar>(xy_index.x(),cell_limits.num_y_cells - 1 - xy_index.y()) = 255;
      } 
      else
        mat_occupancy_grid.at<uchar>(xy_index.x(),cell_limits.num_y_cells - 1 - xy_index.y()) = 0;
    }
  }

  Mat mat_dilate;
  Mat mat_erode;

  Mat mat_counters = Mat::zeros(cell_limits.num_x_cells ,cell_limits.num_y_cells,CV_8UC1 );
  vector<vector<Point> > contours;

  Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

  dilate(mat_occupancy_grid, mat_dilate, element);
  erode(mat_dilate, mat_erode, element);

  findContours( mat_erode, contours, 
        CV_RETR_EXTERNAL , CV_CHAIN_APPROX_TC89_L1  );

  int shipRow = occupancy_grid->info.origin.position.y;
  int shipCol = occupancy_grid->info.origin.position.x;

  stringstream ss;
  string s_yaw;
  ss << yaw;
  ss >> s_yaw;
  ss.clear();

  string SENS = "$SENS;0,0;" + s_yaw + ",0,0,0,0,0,0,0,0;";

  QUdpSocket *udp;
  udp = new QUdpSocket();



  for(int i = 0; i < contours.size(); i++)
  {
    if( contours[i].size() < 10)
      continue;
    for(int j = 0; j < contours[i].size(); j++)
    {
      int col = contours[i][j].x;
      int row = contours[i][j].y;
      if( row >= cell_limits.num_x_cells || col >= cell_limits.num_y_cells)
      {
        LOG(INFO) << " row (height) = "<< x <<" col (width) = "<< y;  
        LOG(INFO) << " cell_limits.num_x_cells (height) = "<< cell_limits.num_x_cells 
                  << " cell_limits.num_y_cells (width) = "<< cell_limits.num_y_cells ;  
      }
      double dist = 0;
      double angle = 0;
      CalAngleAndDist(shipRow,shipCol,row,col,angle,dist);
      dist = dist * 0.1; 
      angle = yaw - angle;
      angle = NormAngle(angle);

      //LOG(INFO) << " shipRow = "<< shipRow  <<" shipCol = "<< shipCol; 
      //LOG(INFO) << " row = "<< row <<" col = "<< col; 
      //LOG(INFO) << " yaw = "<< yaw * 180/PI ; 
      //LOG(WARNING) << " angle = "<< angle * 180/PI <<" dist = "<< dist;  
      string s_dist;
      ss << dist;
      ss >> s_dist;
      ss.clear();

      string s_angle;
      ss << angle;
      ss >> s_angle;
      ss.clear();

      SENS = SENS + s_dist + "," + s_angle + ";";

      //mat_counters.at<uchar>(row,col) = 255;
    }
  }

  string mat_path = "/home/zkma/OUTMAP/"; 
  
  string s_map_num;
  ss << map_num;
  ss >> s_map_num;
  ss.clear();

  map_num++;

  //LOG(INFO) << SENS;
  QByteArray datagram = QString::fromStdString(SENS).toLatin1();
  udp->writeDatagram(datagram, QHostAddress("192.168.1.131"), 5566);


  
  //imwrite( mat_path + s_map_num +".jpg",mat_counters);
  //cartographer_ros::WriteOccupancyGridToPgmAndYaml(*occupancy_grid,"/home/zkma/OUTMAP/outmap");

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
