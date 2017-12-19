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

#ifndef CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include "cartographer/common/math.h"
#include <math.h>


namespace cartographer_ros {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
 public:
  explicit SensorBridge(
      const string& tracking_frame, double lookup_transform_timeout_sec,
      tf2_ros::Buffer* tf_buffer,
      ::cartographer::mapping::TrajectoryBuilder* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  void HandleOdometryMessage(const string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleImuMessage(const string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(const string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      const string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(const string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  const TfBridge& tf_bridge() const;

 private:
  void HandleRangefinder(const string& sensor_id,
                         const ::cartographer::common::Time time,
                         const string& frame_id,
                         const ::cartographer::sensor::PointCloud& ranges);

  const TfBridge tf_bridge_;
  ::cartographer::mapping::TrajectoryBuilder* const trajectory_builder_;

  bool first_tag_gps_ = true;
  Eigen::Quaterniond first_time_orientiation_ = Eigen::Quaterniond(1.0,0,0,0);
  Eigen::Quaterniond real_time_orientiation_ = Eigen::Quaterniond(1.0,0,0,0);
  Eigen::Quaterniond msg_orientiation_ = Eigen::Quaterniond(1.0,0,0,0);

  bool first_tag_imu_ = true;
  double first_lat_ = 0;
  double first_lon_ = 0;
  double real_time_lat_ = 0;
  double real_time_lon_ = 0;

};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
