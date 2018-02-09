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

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/range_data.h"

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"

#include <iostream>
#include <fstream> 


using namespace std; 
using namespace cv;


namespace cartographer {
namespace mapping_2d {

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      odometry_state_tracker_(options_.num_odometry_states()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

sensor::RangeData LocalTrajectoryBuilder::TransformAndFilterRangeData(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped = sensor::CropRangeData(
      sensor::TransformRangeData(range_data, tracking_to_tracking_2d),
      options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFiltered(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFiltered(cropped.misses, options_.voxel_filter_size())};
}

void LocalTrajectoryBuilder::ScanMatch(
    common::Time time, const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::RangeData& range_data_in_tracking_2d,
    transform::Rigid3d* pose_observation) {
  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid2d pose_prediction_2d = //tracking_2d_to_map  [x,y,r]
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction_2d;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  //const sensor::PointCloud filtered_point_cloud_in_tracking_2d =
  //    adaptive_voxel_filter.Filter(range_data_in_tracking_2d.returns);
  const sensor::PointCloud filtered_point_cloud_in_tracking_2d = range_data_in_tracking_2d.returns;
  //mnf returns and misses :=  RELATE TO pose_now
  double score_real_time = 0;
  if (options_.use_online_correlative_scan_matching()) {
    score_real_time = real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        matching_submap->probability_grid(), &initial_ceres_pose);
  }
  // mnf return best_candidate.score
  //LOG(WARNING) << "score_real_time" << score_real_time;

  transform::Rigid2d tracking_2d_to_map;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction_2d, initial_ceres_pose,
                            filtered_point_cloud_in_tracking_2d,
                            matching_submap->probability_grid(),
                            &tracking_2d_to_map, &summary);
  // mnf sumary could be used for judgement

  *pose_observation =
      transform::Embed3D(tracking_2d_to_map) * tracking_to_tracking_2d;
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalRangeData(
    const common::Time time, const sensor::RangeData& range_data) {
  // Initialize IMU tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeImuTracker(time);
  }

  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "ImuTracker not yet initialized.";
    return nullptr;
  }

  Predict(time);


  if (num_accumulated_ == 0) {
    first_pose_estimate_ = pose_estimate_.cast<float>();
    accumulated_range_data_ =
        sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}};
  }

  const transform::Rigid3f tracking_delta =
      first_pose_estimate_.inverse() * pose_estimate_.cast<float>();

  //LOG(INFO) << "tracking_delta"<<tracking_delta;


  const sensor::RangeData range_data_in_first_tracking =
      sensor::TransformRangeData(range_data, tracking_delta);
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (const Eigen::Vector3f& hit : range_data_in_first_tracking.returns) {
    const Eigen::Vector3f delta = hit - range_data_in_first_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit);
      } else {
        accumulated_range_data_.misses.push_back(
            range_data_in_first_tracking.origin +
            options_.missing_data_ray_length() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.scans_per_accumulation()) {
    num_accumulated_ = 0;
    return AddAccumulatedRangeData(
        time, sensor::TransformRangeData(accumulated_range_data_,
                                         tracking_delta.inverse()));
  }

  //mnf
  // origin:= first_pose_estimate_ RELATE TO pose_now
  // (0,0) := pose_now
  // returns and misses :=  RELATE TO pose_now

  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const sensor::RangeData& range_data) {
  const transform::Rigid3d odometry_prediction =
      pose_estimate_ * odometry_correction_;

  //mnf record returns for choose MODE_A and MODE_B
  int num_returns = range_data.returns.size();
  returns_now_ = num_returns;
  returns_pre_ = returns_now_;


      //odometry_correction_ = transform::Rigid3d::Identity();

  const transform::Rigid3d model_prediction = pose_estimate_;

  const transform::Rigid3d& pose_prediction = odometry_prediction;


  //mnf use imu

  // Computes the rotation without yaw, as defined by GetYaw().
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());
  //mnf tracking_to_tracking_2d = map_to_tracking_2d * tracking_to_map(local);

  const sensor::RangeData range_data_in_tracking_2d =
      TransformAndFilterRangeData(tracking_to_tracking_2d.cast<float>(),
                                  range_data);

  if (range_data_in_tracking_2d.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            range_data_in_tracking_2d, &pose_estimate_);
  odometry_correction_ = transform::Rigid3d::Identity();

  if (!odometry_state_tracker_.empty()) {
    // We add an odometry state, so that the correction from the scan matching
    // is not removed by the next odometry data we get.
    odometry_state_tracker_.AddOdometryState(
        {time, odometry_state_tracker_.newest().odometer_pose,
         odometry_state_tracker_.newest().state_pose *
             odometry_prediction.inverse() * pose_estimate_});
  }


  // Improve the velocity estimate.
  if (last_scan_match_time_ > common::Time::min() &&
      time > last_scan_match_time_  && times_ != (mode_c_times_-1) ) {
    const double delta_t = common::ToSeconds(time - last_scan_match_time_);
    // This adds the observed difference in velocity that would have reduced the
    // error to zero.
    velocity_estimate_ += (pose_estimate_.translation().head<2>() -
                           model_prediction.translation().head<2>()) /
                          delta_t; 
    last_scan_match_time_ = time_;                    
  }
  //last_scan_match_time_ = time_;     //mnf where bug for features_map (MODE_A)

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = pose_estimate_.translation();
  pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      pose_estimate_.rotation());

  const transform::Rigid3d tracking_2d_to_map =
      pose_estimate_ * tracking_to_tracking_2d.inverse();
  last_pose_estimate_ = {
      time, pose_estimate_,
      sensor::TransformPointCloud(range_data_in_tracking_2d.returns,
                                  tracking_2d_to_map.cast<float>())};

  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d))) {
    return nullptr;
  }

  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (std::shared_ptr<Submap> submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(
      TransformRangeData(range_data_in_tracking_2d,
                         transform::Embed3D(pose_estimate_2d.cast<float>())));

  //LOG(INFO) << "pose_estimate_2d"<<pose_estimate_2d;
  //LOG(INFO) << "last_pose_estimate_"<<last_pose_estimate_.pose;

  //mnf 1.get insertion_submaps 2.InsertRangeData ???  ---> ptr

  return common::make_unique<InsertionResult>(InsertionResult{
      time, std::move(insertion_submaps), tracking_to_tracking_2d,
      range_data_in_tracking_2d, pose_estimate_2d});
}

const LocalTrajectoryBuilder::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity,const Eigen::Quaterniond& orientiation) {//mnf
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  if (!pose_tracker_) {
    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
            orientation_model_variance = 1e-8,
            position_model_variance = 1e-8,
            velocity_model_variance = 1e-8,
            imu_gravity_time_constant = 100,
            imu_gravity_variance = 1e-9,
            num_odometry_states = 1,
        }
        )text");

    const kalman_filter::proto::PoseTrackerOptions options =
        kalman_filter::CreatePoseTrackerOptions(parameter_dictionary.get());

    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options,time);
  }

  InitializeImuTracker(time);

  Predict(time);

  real_time_orientiation_ = orientiation;


  imu_tracker_->AddImuLinearAccelerationObservation(linear_acceleration,real_time_orientiation_);
  imu_tracker_->AddImuAngularVelocityObservation(angular_velocity);

  pose_tracker_->AddImuLinearAccelerationObservation(
      time,linear_acceleration,real_time_orientiation_);
  pose_tracker_->AddImuAngularVelocityObservation(time,angular_velocity);
}


cv::Mat gps_out_ori = cv::Mat::zeros(1500,1500,CV_8UC1);
cv::Mat gps_out_kalman = cv::Mat::zeros(1500,1500,CV_8UC1);
int tim = 0;

void LocalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& odometer_pose_with_rtk) {
  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker we do not want to call Predict().
    LOG(INFO) << "ImuTracker not yet initialized.";
    return;
  }
  if (!pose_tracker_) {
    LOG(INFO) << "PoseTracker not yet initialized.";
    return;
  }
  Predict(time);

  Eigen::Vector3d  odometer_pose_translation = odometer_pose_with_rtk.translation();
  double rtk = odometer_pose_translation.z();
  transform::Rigid3d odometer_pose = transform::Rigid3d({odometer_pose_translation.x(),odometer_pose_translation.y(),0},
          {1.0,0,0,0});

  pose_tracker_->AddPoseObservation(
      time, odometer_pose,
      Eigen::Matrix<double, 6, 6>::Identity() * 1e-6);

  transform::Rigid3d actual;
  kalman_filter::PoseCovariance covariance;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time , &actual, &covariance);

  transform::Rigid3d odometer_pose_with_imu;
  //mnf use kalman 
  if(rtk == 1)
  {
    odometer_pose_with_imu = transform::Rigid3d(actual.translation(),imu_tracker_->orientation());
  }
  else
  {
    odometer_pose_with_imu = transform::Rigid3d(odometer_pose.translation(),imu_tracker_->orientation());
  }
  
//kalman mat whrite  
/*
  Eigen::Vector3d  actual_t = actual.translation();
  Eigen::Vector3d  gps_t = odometer_pose.translation();

  double actual_x = actual_t.x();
  double actual_y = actual_t.y();

  double odometer_pose_x = gps_t.x();
  double odometer_pose_y = gps_t.y();

  int gps_lat = (int)(odometer_pose_x*10 +500 );
  int gps_lon = (int)(odometer_pose_y*10 +500 );
  gps_out_ori.at<uchar>(gps_lat,gps_lon) = 255;

  int gps_lat_kal = (int)(actual_x*10 +500 );
  int gps_lon_kal = (int)(actual_y*10 +500 );
  gps_out_kalman.at<uchar>(gps_lat_kal,gps_lon_kal) = 255;

  imwrite("/home/zkma/OUTMAP2/gps_out_ori3.jpg",gps_out_ori);
  imwrite("/home/zkma/OUTMAP2/gps_out_kalman3.jpg",gps_out_kalman);
*/ 

  if (!odometry_state_tracker_.empty()) {
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose_with_imu;

    //transform::Rigid3d odometer_pose_with_imu = transform::Rigid3d(odometer_pose.translation(),real_time_orientiation_);
    
    const transform::Rigid3d new_pose = previous_odometry_state.state_pose * delta;
    
    double transform_x = odometer_pose.translation().x() - 
                    pose_estimate_.translation().x(); 

    double transform_y = odometer_pose.translation().y() - 
                    pose_estimate_.translation().y();

    double dist = transform_x * transform_x + transform_y * transform_y;

    //LOG(WARNING) <<  " odometer_pose.translation() " << odometer_pose.translation().x() <<"and"<< odometer_pose.translation().y();
    //LOG(WARNING) <<  " pose_estimate_.translation() " << pose_estimate_.translation().x() <<"and"<< pose_estimate_.translation().y();
    //LOG(WARNING) <<  " dist = " << dist<< " times_ = "<<times_; 

/*
    if(times_ > 1)
    {
      times_--;
      MODE = MODE_C;
    }
    else
    {
      if(dist > mode_c_distance_)
      {
        times_ = mode_c_times_;
        MODE = MODE_C;
      }
      else
      {
        if( (returns_now_ > mode_b_threshold_returns_) && (returns_pre_ > mode_b_threshold_returns_) )
        {
          MODE = MODE_B;
        }
        else
        {
          MODE = MODE_B;
        }
      }
    }
*/

    if((returns_now_ > mode_b_threshold_returns_) && (returns_pre_ > mode_b_threshold_returns_))
    {
      MODE = MODE_A;
    }
    else
    {
      if(dist > mode_c_distance_)
      {
        times_ = mode_c_times_;
        MODE = MODE_C;
      }
      else
      {
        if( times_ > 1 )
        {
          MODE = MODE_C;
          times_--;
        }
        else
        {
          MODE = MODE_B;
        }
      }
    }

    if( MODE == MODE_A )
    {
      odometry_correction_ = transform::Rigid3d::Identity();
      LOG(WARNING) <<  " MODE == MODE_A " ; 
    }
    if( MODE == MODE_B )
    {
      odometry_correction_ = pose_estimate_.inverse() * new_pose;
      //LOG(WARNING) <<  " MODE == MODE_B " ; 
    }
    if( MODE == MODE_C )
    {
      odometry_correction_ = pose_estimate_.inverse() * odometer_pose_with_imu;
    }

  }
  odometry_state_tracker_.AddOdometryState(
    {time, odometer_pose_with_imu, pose_estimate_ * odometry_correction_});

}


void LocalTrajectoryBuilder::InitializeImuTracker(const common::Time time) {
  if (imu_tracker_ == nullptr) {
    imu_tracker_ = common::make_unique<mapping::ImuTracker>(
        options_.imu_gravity_time_constant(), time);
  }
}

void LocalTrajectoryBuilder::Predict(const common::Time time) {
  CHECK(imu_tracker_ != nullptr);
  CHECK_LE(time_, time);
  const double last_yaw = transform::GetYaw(imu_tracker_->orientation());
  //LOG(INFO) << "before advance last_yaw" << last_yaw;
  imu_tracker_->Advance(time);
  const double aft_last_yaw = transform::GetYaw(imu_tracker_->orientation());
  //LOG(INFO) << "after advance last_yaw" << aft_last_yaw;
  if (time_ > common::Time::min()) {
    const double delta_t = common::ToSeconds(time - time_);
    // Constant velocity model.
    const Eigen::Vector3d translation =
        pose_estimate_.translation() +
        delta_t *
            Eigen::Vector3d(velocity_estimate_.x(), velocity_estimate_.y(), 0.);
    // Use the current IMU tracker roll and pitch for gravity alignment, and
    // apply its change in yaw.
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(
            transform::GetYaw(pose_estimate_.rotation()) - last_yaw,
            Eigen::Vector3d::UnitZ()) *
        imu_tracker_->orientation();
    //LOG(WARNING) <<  " BF " << pose_estimate_.translation().x() <<"and"<< pose_estimate_.translation().y();    
    pose_estimate_ = transform::Rigid3d(translation, rotation);
    //LOG(WARNING) <<  " AFT " << pose_estimate_.translation().x() <<"and"<< pose_estimate_.translation().y();    
  }
  time_ = time;
}

}  // namespace mapping_2d
}  // namespace cartographer
