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

#include "cartographer_ros/sensor_bridge.h"

#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id) {

  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace


//2arcsin{[sin((lat_dif/2)^2) + cos(lat1)*cos(lat2)*sin((lon_dif/2)^2)]^0.5}
void ConvertToPoseFromLatAndLon(double lat_from, double lat_to,
                            double lon_from, double lon_to , double* pose)
{
  //printf("BF lat_from  %.10lf \n",lat_from);
  //printf("BF lat_to  %.10lf \n",lat_to);
  
  lat_from = ::cartographer::common::DegToRad(lat_from);
  lat_to = ::cartographer::common::DegToRad(lat_to);
  lon_from = ::cartographer::common::DegToRad(lon_from);
  lon_to = ::cartographer::common::DegToRad(lon_to);

  //printf("AFT lat_from  %.10lf \n",lat_from);
  //printf("AFT lat_to  %.10lf \n",lat_to);

  double lat_dif = (fabs(lat_to - lat_from));
  double lon_dif = (fabs(lon_to - lon_from));

  //printf("lat_dif  %.10lf \n",lat_dif);
  //printf("lon_dif  %.10lf \n",lon_dif);

  double pose_lat = lat_dif * 6378.137 * 1000; //KM -> M
  double pose_lon = 2 * asin(sqrt((cos(lat_from)*
                    cos(lat_to)*sin(pow(lon_dif/2,2)))));
  pose_lon = pose_lon * 6378.137 * 1000;

  if(lat_from > lat_to)
    pose_lat = - pose_lat;
  if(lon_from > lon_to)
    pose_lon = - pose_lon;

  pose[0] = pose_lat;
  pose[1] = -pose_lon;
}


SensorBridge::SensorBridge(
    const string& tracking_frame, const double lookup_transform_timeout_sec,
    tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

void SensorBridge::HandleOdometryMessage(
    const string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  //LOG(INFO) << "warning@@@@@@@@@@@@@@@@@@@@" << msg->header.frame_id << "end";
   if (sensor_to_tracking != nullptr) {
    //mnf
    real_time_lat_ = (double)msg->pose.pose.position.x;
    real_time_lon_ = (double)msg->pose.pose.position.z;
    
    //printf("lat %.10lf \n",real_time_lat_);
 
    if(first_tag_gps_)
    {
      if((real_time_lat_!= 0)||(real_time_lon_!= 0))
      {
        first_lat_ = real_time_lat_;
        first_lon_ = real_time_lon_;
        first_tag_gps_ = false;
      }
    }

    double relative_pose[2] = {0,0};

    ConvertToPoseFromLatAndLon(first_lat_,real_time_lat_,
              first_lon_,real_time_lon_,relative_pose);
    printf("senser bridge --> lat dist  %.10lf \n",relative_pose[0]);
    printf("senser bridge --> lon dist  %.10lf \n",relative_pose[1]);
    
    trajectory_builder_->AddOdometerData(
        sensor_id, time,
        carto::transform::Rigid3d({relative_pose[0],relative_pose[1],0},
          {1.0,0,0,0}));

    //mnf
    /*
    trajectory_builder_->AddOdometerData(
        sensor_id, time,
        ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse());
    */
  }

  //LOG(INFO) <<"HERE IS void SensorBridge::HandleOdometryMessage";
}

void SensorBridge::HandleImuMessage(const string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking != nullptr) {
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear acceleration into the tracking frame will "
           "otherwise be imprecise.";
  
  msg_orientiation_ = ToEigen(msg->orientation);

  if(first_tag_imu_)
  {
    first_time_orientiation_ = msg_orientiation_;
    first_tag_imu_ = false;
  }

  if((msg_orientiation_.x() != 0) || (msg_orientiation_.y() != 0) || (msg_orientiation_.z() != 0))
    real_time_orientiation_ = first_time_orientiation_.inverse() * msg_orientiation_ ;
  // mnf first_time_orientiation_  --> T01
  // msg_orientiation_ --> T02
  // real_time_orientiation_ -->T12 = T02 * T01^(-1)

  trajectory_builder_->AddImuData(
      sensor_id, time,
      sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity),
      sensor_to_tracking->rotation() * real_time_orientiation_); //mnf
  }
}

void SensorBridge::HandleLaserScanMessage(
    const string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    ToPointCloudWithIntensities(*msg).points);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    ToPointCloudWithIntensities(*msg).points);
}

void SensorBridge::HandlePointCloud2Message(
    const string& sensor_id, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  carto::sensor::PointCloud point_cloud;
  for (const auto& point : pcl_point_cloud) {
    point_cloud.emplace_back(point.x, point.y, point.z);
  }
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    point_cloud);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleRangefinder(const string& sensor_id,
                                     const carto::common::Time time,
                                     const string& frame_id,
                                     const carto::sensor::PointCloud& ranges) {
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddRangefinderData(
        sensor_id, time, sensor_to_tracking->translation().cast<float>(),
        carto::sensor::TransformPointCloud(ranges,
                                           sensor_to_tracking->cast<float>()));
  }

}

}  // namespace cartographer_ros
