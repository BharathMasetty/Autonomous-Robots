//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include <sensor_msgs/Image.h>
#include <visualization/visualization.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include "slam.h"

#include "vector_map/vector_map.h"
using namespace gtsam;
using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector2d;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM(ros::NodeHandle *node_handle):
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    node_handle_(node_handle){

    node_handle_->param(kLaserVarianceParamName, laser_variance_, kDefaultLaserVariance);

    node_handle_->param(kLaserUpdateOdomLocDifferenceParamName, laser_update_odom_loc_difference_,
                        kDefaultLaserUpdateOdomLocDifference);
    node_handle_->param(kLaserUpdateOdomAngleDifferenceParamName, laser_update_odom_angle_difference_,
                        kDefaultLaserUpdateOdomAngleDifference);

    node_handle_->param(kMotionModelRotErrorFromRotParamName, motion_model_rot_error_from_rot_,
                        kDefaultMotionModelRotErrorFromRot);
    node_handle_->param(kMotionModelRotErrorFromTranslParamName, motion_model_rot_error_from_transl_,
                        kDefaultMotionModelRotErrorFromTransl);
    node_handle_->param(kMotionModelTranslErrorFromRotParamName, motion_model_transl_error_from_rot_,
                        kDefaultMotionModelTranslErrorFromRot);
    node_handle_->param(kMotionModelTranslErrorFromTranslParamName, motion_model_transl_error_from_transl_,
                        kDefaultMotionModelTranslErrorFromTransl);

    node_handle_->param(kPoseSearchStdDevMultiplierParamName, pose_search_std_dev_multiplier_,
                        kDefaultPoseSearchStdDevMultiplier);

    node_handle_->param(kPoseEvalTranslIncrementParamName, pose_eval_transl_increment_,
                        kDefaultPoseEvalTranslIncrement);
    node_handle_->param(kPoseEvalRotIncrementParamName, pose_eval_rot_increment_, kDefaultPoseEvalRotIncrement);

    node_handle_->param(kRasterGridIncrementParamName, raster_grid_increment_, kDefaultRasterGridIncrement);
    node_handle_->param(kRasterGridSizeParamName, raster_grid_size_, kDefaultRasterGridSize);

    node_handle_->param(kUseGTSAMParamName, use_gtsam_, kDefaultUseGTSAMConfig);

    node_handle_->param(kNonSuccessiveMaxPosDifferenceParamName, non_successive_max_pos_difference_,
                        kDefaultNonSuccessiveMaxPosDifference);

    raster_grid_center_offset_ = (0.5 * raster_grid_increment_) - (0.5 * raster_grid_size_);

    raster_rows_and_cols_ = ceil(raster_grid_size_ / raster_grid_increment_);
    raster_mat_with_log_probs_.resize(raster_rows_and_cols_, raster_rows_and_cols_);

    image_pub_ =
            node_handle_->advertise<sensor_msgs::Image>("raster_img", 1);
    
    graph_ = new NonlinearFactorGraph();
    // Adding prior noise to the initial pose
    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = 
	    noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-6));
    std::cout << "All clear" << std::endl;
    graph_->add(PriorFactor<Pose2>(0, priorMean, priorNoise));
    initialEstimates_.insert(0, Pose2(0, 0, 0));
}

void SLAM::publishTrajectory(amrl_msgs::VisualizationMsg &vis_msg) {
    for (const auto &trajectory_point_with_cov : trajectory_estimates_) {
        std::pair<Vector2f, float> trajectory_point = trajectory_point_with_cov.first;

        Vector2f second_point(kTrajectoryPlotLineSegName * cos(trajectory_point.second), kTrajectoryPlotLineSegName * sin(trajectory_point.second));
        visualization::DrawLine(trajectory_point.first,
                                trajectory_point.first + second_point,
                                kTrajectoryColor, vis_msg);
    }

    for (const auto &odom_pos : odom_only_estimates_) {
        Vector2f second_point(kTrajectoryPlotLineSegName * cos(odom_pos.second), kTrajectoryPlotLineSegName * sin(odom_pos.second));
        visualization::DrawLine(odom_pos.first,
                                odom_pos.first + second_point,
                                kOdometryEstColor, vis_msg);
    }
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
    std::pair<Eigen::Vector2f, float> locInfo;
    if (trajectory_estimates_.empty()) {
        Vector2f empty_vec(0, 0);
        locInfo = std::make_pair(empty_vec, 0);
    } else {
        locInfo = trajectory_estimates_.back().first;
    }
    
    // Return the latest pose estimate of the robot plus whatever odometry has not yet been incorporated into the
    // trajectory.
    Eigen::Vector2f& loc_ = *loc;
    float& angle_ = *angle;

    Eigen::Vector2f unrotated_odom_est_loc_displ = prev_odom_loc_ - odom_loc_at_last_laser_align_;
    float odom_est_angle_displ = math_util::AngleDiff(prev_odom_angle_, odom_angle_at_last_laser_align_);
    Eigen::Rotation2Df rotate(-1 * prev_odom_angle_);
    Vector2f odom_est_loc_displ = rotate * unrotated_odom_est_loc_displ;

    Rotation2Df eig_rotation(locInfo.second);
    Vector2f rotated_offset = eig_rotation * odom_est_loc_displ;

    loc_ = locInfo.first + rotated_offset;
    angle_ = locInfo.second + odom_est_angle_displ;
    ROS_INFO_STREAM("Pose " << loc_.x() << ", " << loc_.y() << ", " << angle_);
}

bool SLAM::shouldProcessLaser() {
    float angle_diff = math_util::AngleDist(prev_odom_angle_, odom_angle_at_last_laser_align_);
    float loc_diff = (prev_odom_loc_ - odom_loc_at_last_laser_align_).norm();

    if ((loc_diff > laser_update_odom_loc_difference_) || (angle_diff > laser_update_odom_angle_difference_)) {
        ROS_INFO_STREAM("Should process laser");
        return true;
    } else {
        return false;
    }
}

void SLAM::convertRangesToPointCloud(const vector<float>& ranges, const float &angle_min, const float &angle_max,
                                     const float &range_max, vector<Eigen::Vector2f> &point_cloud) {

    const int num_rays = ranges.size();
    point_cloud.clear();
    point_cloud.reserve(num_rays);
    double angle_inc = (angle_max - angle_min) / (num_rays - 1);
    for (int i=0; i < num_rays; i++) {
        float range = ranges[i];
        // Exclude points at or greater than range max because they don't correspond to an actual obstacle in the world
        if (range < range_max) {
            float curr_angle = angle_min + (i * angle_inc);
            Vector2f cloud_point(range * cos(curr_angle) + kBaselinkToLaserOffsetX, range * sin(curr_angle));
            point_cloud.push_back(cloud_point);
        }
    }
}

void SLAM::publishRasterAsImage() {

    sensor_msgs::Image image;

    image.header.frame_id = "raster";
    image.is_bigendian = false;
    image.height = raster_rows_and_cols_;
    image.width = raster_rows_and_cols_;
    image.step = raster_rows_and_cols_;

    // Find the minimum and maximum values from the raster (used to scale)
    double max_raster = -std::numeric_limits<double>::infinity();
    double min_raster = std::numeric_limits<double>::infinity();
    for (int x = 0; x < raster_rows_and_cols_; x++) {
        for (int y = 0; y < raster_rows_and_cols_; y++) {
            double raster_val = raster_mat_with_log_probs_(y, x);
            if (max_raster < raster_val) {
                max_raster = raster_val;
            }
            if (min_raster > raster_val) {
                min_raster = raster_val;
            }
        }
    }

    // Convert the raster values into the range 0-255 and add them to the image
    for (int y = 0; y < raster_rows_and_cols_; y++) {
        for (int x = 0; x < raster_rows_and_cols_; x++) {
            double raster_val = raster_mat_with_log_probs_(y, x);
            double scaled_double = ((std::exp(raster_val) - std::exp(min_raster)) / (std::exp(max_raster) - std::exp(min_raster))) * 255.0;
            uint8_t scaled = scaled_double;
            image.data.emplace_back(scaled);
        }
    }

    image.encoding = "mono8";
    image_pub_.publish(image);
}

double SLAM::computeLogProbForRelativePose(const vector<Vector2f> &rotated_current_scan,
                                           const Vector2f &position_offset) {
    double log_prob = 0;

    for (const Vector2f &curr_scan_point : rotated_current_scan) {
        Vector2f transformed_scan_point = curr_scan_point + position_offset;
        int lookup_column = floor((transformed_scan_point.x() - raster_grid_center_offset_) / raster_grid_increment_);
        int lookup_row = floor((transformed_scan_point.y() - raster_grid_center_offset_) / raster_grid_increment_);

        // Ignoring points that don't fall within the raster range.
        if ((lookup_row >= 0) && (lookup_row < raster_rows_and_cols_)
                && (lookup_column >= 0) && (lookup_column < raster_rows_and_cols_)) {
            log_prob += raster_mat_with_log_probs_(lookup_row, lookup_column);
        }
    }
    return log_prob;
}

double SLAM::computeMotionLogProbForRelativePose(const Eigen::Vector2f &position_offset,  const float &angle_offset,
                                                 const Eigen::Vector2f &odom_position_offset,
                                                 const float &odom_angle_offset) {
    double motionLogProb;

    float delta_rot_1 = atan2(odom_position_offset.y(), odom_position_offset.x());
    float delta_trans = odom_position_offset.norm();
    float delta_rot_2 = odom_angle_offset - delta_rot_1;

    std::pair<Vector2f, float> prev_loc_info;
    if (trajectory_estimates_.empty()) {
        prev_loc_info = std::make_pair(Vector2f(0, 0), 0.0);
    } else {
        prev_loc_info = trajectory_estimates_.back().first;
    }

    float delta_rot_1_hat = atan2(position_offset.y(), position_offset.x());
    float delta_trans_hat = position_offset.norm();
    float delta_rot_2_hat = angle_offset - delta_rot_1_hat;

    float delta_rot_1_std_dev = motion_model_rot_error_from_rot_ * delta_rot_1_hat +
            motion_model_rot_error_from_transl_*delta_trans_hat;
    float delta_trans_std_dev = motion_model_transl_error_from_rot_ * (delta_rot_1_hat + delta_rot_2_hat) +
            motion_model_transl_error_from_transl_ * delta_trans_hat;
    float delta_rot_2_std_dev = motion_model_rot_error_from_rot_ * delta_rot_2_hat +
            motion_model_rot_error_from_transl_ * delta_trans_hat;

    motionLogProb  = -std::pow(delta_rot_1-delta_rot_1_hat,2)/(2*std::pow(delta_rot_1_std_dev, 2));
    motionLogProb += -std::pow(delta_trans-delta_trans_hat,2)/(2*std::pow(delta_trans_std_dev ,2));
    motionLogProb += -std::pow(delta_rot_2-delta_rot_2_hat,2)/(2*std::pow(delta_rot_2_std_dev ,2));

    return motionLogProb;
}

void SLAM::computeLogProbsForRotatedScans(const vector<Vector2f> &rotated_current_scan, const float &angle,
                                          const vector<float> &possible_x_offsets,
                                          const vector<float> &possible_y_offsets,
                                          const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset,
                                          const bool &compute_motion_likelihood,
                                          vector<RelativePoseResults> &log_prob_results) {

    // Scan will have already been rotated by the time this is called, so we just have to translate the points
    for (const float &x_offset : possible_x_offsets) {
        for (const float &y_offset : possible_y_offsets) {
            Vector2f position_offset(x_offset, y_offset);
            double log_prob = computeLogProbForRelativePose(rotated_current_scan, position_offset);
            RelativePoseResults result;
            result.location_offset_ = position_offset;
            result.rotation_offset_ = angle;
            result.obs_log_probability_ = log_prob;
            if (compute_motion_likelihood) {
                double motion_log_prob = computeMotionLogProbForRelativePose(position_offset, angle,
                                                                             odom_position_offset, odom_angle_offset);
                result.motion_log_probability_ = motion_log_prob;
            }
            log_prob_results.emplace_back(result);
        }
    }
}

void SLAM::computeLogProbsForPoseGrid(const vector<Vector2f> &current_scan, const Vector2f &odom_position_offset,
                                      const float &odom_angle_offset, const bool &compute_motion_likelihood, const bool &use_odom_for_search_range,
                                      vector<RelativePoseResults> &relative_pose_results) {

    float transl_std_dev;
    float rot_std_dev;
    if (use_odom_for_search_range) {
        transl_std_dev = (motion_model_transl_error_from_transl_ * (odom_position_offset.norm())) +
                               (motion_model_transl_error_from_rot_ * fabs(odom_angle_offset));
        rot_std_dev = (motion_model_rot_error_from_transl_ * (odom_position_offset.norm())) +
                            (motion_model_rot_error_from_rot_ * fabs(odom_angle_offset));
    } else {
        ROS_INFO_STREAM("Not using motion likelihood");
        transl_std_dev = 0.25;
        rot_std_dev = 0.1; // TODO maybe replace this with cov from original estimate plus odom
    }
	
    // We're searching all poses within some confidence interval of the standard deviation
    // Compute the maximum translation offset from the odometry estimate that we should search
    // and then round up so it is a multiple of the translation search increment
    // I.e. if our desired confidence is 95%, then we'll search within 2 standard deviations on each side of the
    // odometry position
    float transl_search_target_offset = pose_search_std_dev_multiplier_ * transl_std_dev;
    float transl_search_max = pose_eval_transl_increment_ * ceil(transl_search_target_offset / pose_eval_transl_increment_);

    float transl_offset = -1 * transl_search_max;
    vector<float> possible_x_offsets;
    vector<float> possible_y_offsets;

    while (transl_offset <= transl_search_max) {
        float eval_x = transl_offset + odom_position_offset.x();
        float eval_y = transl_offset + odom_position_offset.y();

        possible_x_offsets.emplace_back(eval_x);
        possible_y_offsets.emplace_back(eval_y);
        transl_offset += pose_eval_transl_increment_;
    }

    // Compute the maximum rotation offset from the odometry estimate that we should search and then round it up
    // so it is a multiple of the rotation search increment. Bound the search to +/- pi (so we don't wrap around)
    // I.e. if our desired confidence is 95%, then we'll search within 2 standard deviations on each side of the
    // rotation that is estimated by the odometry
    float rot_search_target_offset = pose_search_std_dev_multiplier_ * rot_std_dev;
    float rot_search_max = std::min(M_PI, (double) pose_eval_rot_increment_ * ceil(rot_search_target_offset / pose_eval_rot_increment_));

    vector<float> possible_rotations;
    float rot_offset = -1 * rot_search_max;
    while (rot_offset <= rot_search_max) {
        possible_rotations.emplace_back(math_util::AngleMod(rot_offset + odom_angle_offset));
        rot_offset += pose_eval_rot_increment_;
    }

    ROS_INFO_STREAM("Rotation min, max " << possible_rotations.front() << ", " << possible_rotations.back());
    ROS_INFO_STREAM("X min, max " << possible_x_offsets.front() << ", " << possible_x_offsets.back());
    ROS_INFO_STREAM("Y min, max " << possible_y_offsets.front() << ", " << possible_y_offsets.back());

    relative_pose_results.clear();
    size_t reserve_size = possible_rotations.size() * possible_y_offsets.size() * possible_x_offsets.size();
    relative_pose_results.reserve(reserve_size);

    // Iterate over the angles in the outermost loop so that we can rotate the scans only once for each angle
    for (const float &rot_angle : possible_rotations) {
        Rotation2Df eig_rotation(rot_angle);
        vector<Vector2f> rotated_scan;
        rotated_scan.reserve(current_scan.size());
        for (const Vector2f &point : current_scan) {
            rotated_scan.emplace_back(eig_rotation * point);
        }
        computeLogProbsForRotatedScans(rotated_scan, rot_angle, possible_x_offsets, possible_y_offsets,
                                       odom_position_offset, odom_angle_offset, compute_motion_likelihood,
                                       relative_pose_results);
    }
}

void SLAM::getMaximumLikelihoodScanOffset(const vector<RelativePoseResults> &relative_pose_results,
                                          const bool &use_motion_likelihood, Vector2f &max_likelihood_position_offset,
                                          float &max_likelihood_angular_offset) {
    double max_likelihood = -std::numeric_limits<double>::infinity();
    double totalLogLikelihood;
    for (const RelativePoseResults &pose_result : relative_pose_results) {
        if (use_motion_likelihood) {
            totalLogLikelihood = pose_result.motion_log_probability_ + pose_result.obs_log_probability_;
        } else {
            totalLogLikelihood = pose_result.obs_log_probability_;
        }

        if (totalLogLikelihood > max_likelihood) {
            max_likelihood_position_offset = pose_result.location_offset_;
            max_likelihood_angular_offset = pose_result.rotation_offset_;
            max_likelihood = totalLogLikelihood;
        }
    }
}

void SLAM::ObserveLaserMultipleScansCompared(const std::vector<float> &ranges, float range_min, float range_max, float angle_min,
                        float angle_max) {

    // This version should be used when we're integrating with GTSAM
	  
    // Compute the relative pose of the robot at time t-1 relative to the robot pose at time t
    Vector2f inv_odom_est_displacement_unrotated = odom_loc_at_last_laser_align_ - prev_odom_loc_;
    float inv_odom_est_angle_disp = math_util::AngleDiff(odom_angle_at_last_laser_align_, prev_odom_angle_);
    Eigen::Rotation2Df rotate(-1 * odom_angle_at_last_laser_align_);
    Vector2f inv_odom_est_displacement = rotate * inv_odom_est_displacement_unrotated;

    // Compute the point cloud relative to the robot's current position
    vector<Eigen::Vector2f> current_point_cloud;
    convertRangesToPointCloud(ranges, angle_min, angle_max, range_max, current_point_cloud);

    // Update the rasterized lookup of the current scan
    updateRasterizedLookup(current_point_cloud);

    // Add odometry factor to GTSAM TODO 
    // Not completly sure about this step (!!verify please!!)
    float transl_std_dev = (motion_model_transl_error_from_transl_ * (inv_odom_est_displacement.norm())) +
            (motion_model_transl_error_from_rot_ * fabs(inv_odom_est_angle_disp));
    float rot_std_dev = (motion_model_rot_error_from_transl_ * (inv_odom_est_displacement.norm())) +
            (motion_model_rot_error_from_rot_ * fabs(inv_odom_est_angle_disp));
    noiseModel::Diagonal::shared_ptr odometryNoise =
    	    noiseModel::Diagonal::Sigmas(Vector3(transl_std_dev, transl_std_dev, rot_std_dev));
    
    int index_of_tminus1 = trajectory_estimates_.size();
    int index_of_t = index_of_tminus1 + 1;
    Pose2 odometry(double(inv_odom_est_displacement.x()), double(inv_odom_est_displacement.y()), double(inv_odom_est_angle_disp));
    ROS_INFO_STREAM("Graph interaction 1");
    ROS_INFO_STREAM("Odom " << odometry.x() << ", " << odometry.y() << ", " << odometry.theta());

    graph_->add(BetweenFactor<Pose2>(index_of_t, index_of_tminus1, odometry, odometryNoise));
    ROS_INFO_STREAM("Added factor between " << index_of_t << ", " << index_of_tminus1);
    ROS_INFO_STREAM("Graph interaction 1 done");
    
    // Compute log prob for possible poses
    // This gives likelihood of inverse transform (gives pose of t-1 in t)
    vector<RelativePoseResults> relative_pose_results;
    computeLogProbsForPoseGrid(most_recent_used_scan_, inv_odom_est_displacement, inv_odom_est_angle_disp, false, true, relative_pose_results);

    // Compute the maximum likelihood translation and rotation (provides maximum likelihood pose of the robot at t-1 relative to t)
    Vector2f maximum_likelihood_scan_offset_position(0, 0);
    float maximum_likelihood_scan_offset_angle = 0.0;
    getMaximumLikelihoodScanOffset(relative_pose_results, false, maximum_likelihood_scan_offset_position,
                                   maximum_likelihood_scan_offset_angle);

    // compute covariance
    // compute covariance of t-1 with respect to t (not global covariance like we're using now -- also only use observation, not motion)
//    Eigen::Matrix3d recent_inv_cov = computeRelativeCovariance(relative_pose_results);

    // insert into GTSAM, using cov est and MLE (make sure this is t-1 relative to t)
//    noiseModel::Gaussian::shared_ptr cov =   noiseModel::Gaussian::Covariance(recent_inv_cov);
    Pose2 mle(double(maximum_likelihood_scan_offset_position.x()), double(maximum_likelihood_scan_offset_position.y()), double(maximum_likelihood_scan_offset_angle));
    ROS_INFO_STREAM("Graph interaction 2");
    graph_->add(BetweenFactor<Pose2>(index_of_t, index_of_tminus1, mle, odometryNoise));
//    ROS_INFO_STREAM("Recent inv cov " << recent_inv_cov);
    ROS_INFO_STREAM("Added factor between " << index_of_t << ", " << index_of_tminus1);
    ROS_INFO_STREAM("MLE " << mle.x() << ", " << mle.y() << ", " << mle.theta());
//    ROS_INFO_STREAM("Cov " << cov);
    ROS_INFO_STREAM("Graph interaction 2 done");
    // inserting initial estimate for pose at t
    Vector2f loc_guess_at_t;
    ROS_INFO_STREAM("Trajectory estimates size " << trajectory_estimates_.size());
    Vector2f prev_pose_est(0, 0);
    float prev_angle_est(0);
    if (!trajectory_estimates_.empty()) {
        prev_pose_est = trajectory_estimates_.back().first.first;
        prev_angle_est = trajectory_estimates_.back().first.second;
    }
    loc_guess_at_t = prev_pose_est - inv_odom_est_displacement_unrotated;
    float angle_guess_at_t = prev_angle_est - inv_odom_est_angle_disp;
    Pose2 pose_guess_at_t(double(loc_guess_at_t.x()), double(loc_guess_at_t.y()), double(angle_guess_at_t));
    ROS_INFO_STREAM("Inserting initial estimate");
    initialEstimates_.insert(index_of_t, pose_guess_at_t);
    ROS_INFO_STREAM("Inserting initiali estimate done");

    // If we have more than 1 pose in the trajectory, consider adding a constraint between poses older than this one
    // and the new pose
    if (laser_observations_for_pose_in_trajectory_.size() > 1) {
        Eigen::Matrix3f t_to_t_min_1_transform;

        t_to_t_min_1_transform << cos(inv_odom_est_angle_disp), -sin(inv_odom_est_angle_disp), inv_odom_est_displacement.x(),
                sin(inv_odom_est_angle_disp),  cos(inv_odom_est_angle_disp), inv_odom_est_displacement.y(),
                0,               0,              1;

        Vector2f robot_loc_at_tminus1 = trajectory_estimates_.back().first.first;
        float robot_angle_at_tminus1 = trajectory_estimates_.back().first.second;
        Eigen::Matrix3f map_to_t_min_1_tf;
        map_to_t_min_1_tf << cos(robot_angle_at_tminus1), -sin(robot_angle_at_tminus1), robot_loc_at_tminus1.x(),
                sin(robot_angle_at_tminus1),  cos(robot_angle_at_tminus1), robot_loc_at_tminus1.y(),
                0,               0,              1;

        Eigen::Matrix3f t_to_map_tf;
        t_to_map_tf = t_to_t_min_1_transform * (map_to_t_min_1_tf.inverse());


        for (size_t i = 0; i < laser_observations_for_pose_in_trajectory_.size()-1; i++) {

            // Get initial estimated relative pose from t to i (pose of robot at time i relative to pose of robot at current scan)
	        Vector2f robot_loc_at_i = trajectory_estimates_[i].first.first;
	        float robot_angle_at_i = trajectory_estimates_[i].first.second;

            Eigen::Matrix3f map_to_i_tf;
            map_to_i_tf << cos(robot_angle_at_i), -sin(robot_angle_at_i), robot_loc_at_i.x(),
                    sin(robot_angle_at_i),  cos(robot_angle_at_i), robot_loc_at_i.y(),
                    0,               0,              1;

            Eigen::Matrix3f t_to_i_tf = t_to_map_tf * map_to_i_tf;
            // Find by combining pose of i relative to t-1 and then use odom estimate to get pose of t-1 in t and combine
            Vector2f est_pose_robot_at_time_i_rel_to_t(t_to_i_tf(0, 2), t_to_i_tf(1, 2));
            Eigen::Matrix2f rotation_mat;
            rotation_mat << t_to_i_tf(0, 0), t_to_i_tf(0, 1), t_to_i_tf(1, 0), t_to_i_tf(1, 1);
            Rotation2Df rotation_obj(rotation_mat);
            float est_angle_robot_at_time_i_rel_to_t = rotation_obj.angle();

            // TODO compute this based on pose estimate (don't want to compare poses that won't have significantly overlapping scans)
            bool poses_close_enough_to_compare =
                    est_pose_robot_at_time_i_rel_to_t.norm() < non_successive_max_pos_difference_;

	        if (poses_close_enough_to_compare) {

                vector<RelativePoseResults> relative_pose_results_pose_i;
                computeLogProbsForPoseGrid(laser_observations_for_pose_in_trajectory_[i], est_pose_robot_at_time_i_rel_to_t, est_angle_robot_at_time_i_rel_to_t,
                                           false, false, relative_pose_results_pose_i);

                Vector2f mle_robot_pose_i_rel_to_t(0, 0);
                float  mle_robot_angle_i_rel_to_t = 0.0;
                getMaximumLikelihoodScanOffset(relative_pose_results_pose_i, false, mle_robot_pose_i_rel_to_t,
                                               mle_robot_angle_i_rel_to_t);
                ROS_INFO_STREAM("est offset " << est_pose_robot_at_time_i_rel_to_t << ", " << est_angle_robot_at_time_i_rel_to_t);
                ROS_INFO_STREAM("ML solution " << mle_robot_pose_i_rel_to_t << ", " << mle_robot_angle_i_rel_to_t);
                ROS_INFO_STREAM("t to ti tf " << inv_odom_est_displacement << ", " << inv_odom_est_angle_disp);
                ROS_INFO_STREAM("robot at loc i " << robot_loc_at_i << ", " << robot_angle_at_i);
                ROS_INFO_STREAM("robot at loc t-1 " << robot_loc_at_tminus1 << ", " << robot_angle_at_tminus1);
//
//                // TODO compute covariance of pose at i relative to t (not global covariance like we're using now)
//		        // Uncomment when using
//        //		Eigen::Matrix3d cov_i_rel_to_t = computeRelativeCovariance(relative_pose_results_pose_i);
//                // TODO insert into GTSAM, using cov est and MLE (make sure this is i relative to t)
            	int index_of_i = i+1;
		        Pose2 non_successive_mle(double(mle_robot_pose_i_rel_to_t.x()), double(mle_robot_pose_i_rel_to_t.y()), double(mle_robot_angle_i_rel_to_t));
//        //		noiseModel::Gaussian::shared_ptr cov_i_t=  noiseModel::Gaussian::Covariance(cov_i_rel_to_t);
        		ROS_INFO_STREAM("Graph interaction 3 " << index_of_t << ", " << index_of_i);
        		graph_->add(BetweenFactor<Pose2>(index_of_t, index_of_i, non_successive_mle, odometryNoise));
                ROS_INFO_STREAM("Graph interaction 3 done");
	        }
        }
    }

    // Update the fields for the next laser reading
    laser_observations_for_pose_in_trajectory_.emplace_back(current_point_cloud);
    odom_angle_at_last_laser_align_ = prev_odom_angle_;
    odom_loc_at_last_laser_align_ = prev_odom_loc_;
    most_recent_used_scan_ = current_point_cloud;

    // TODO extract trajectory estimates from GTSAM instead of building them up on our own
    // Replace content of trajectory_estimates_ with revised estimates (MAKE SURE THAT trajectory_estimates_ remains the
    // same size as laser_observations_for_pose_in_trajectory_)
    ROS_INFO_STREAM("Results");
    GaussNewtonParams params;
    params.maxIterations = 100;
    params.verbosity = GaussNewtonParams::Verbosity::VALUES;
    params.errorTol = 1e-5;
    Values result = GaussNewtonOptimizer(*graph_, initialEstimates_, params).optimize();
    ROS_INFO_STREAM("Results done");
    Marginals marginals(*graph_, result);
    ROS_INFO_STREAM("Marginals done");
    // Update Trajectory Estimates vector


    trajectory_estimates_.clear();
    Values::iterator it;
    bool first_iter = true;
    for(it = result.begin(); it!=result.end(); it++){
        if (first_iter) {
            first_iter = false;
            continue;
        }
        int key = it->key;
        double x = result.at<Pose2>(key).x();
        double y = result.at<Pose2>(key).y();
        double theta = result.at<Pose2>(key).theta();
        Vector2d loc(x,y);
        Eigen::Matrix3d marginal_cov = marginals.marginalCovariance(key);
        Eigen::Matrix3f marginal_cov_float = marginal_cov.cast<float>();

        Vector2f loc_float = loc.cast<float>();
        trajectory_estimates_.emplace_back(std::make_pair(std::make_pair(loc_float, (float) theta), marginal_cov_float));
    }
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
    // A new laser scan has been observed. Decide whether to add it as a pose
    // for SLAM. If decided to add, align it to the scan from the last saved pose,
    // and save both the scan and the optimized pose.

    if (!odom_initialized_) {
        ROS_ERROR_STREAM("Odom not initialized. Skipping laser processing.");
        return;
    }

    if (first_scan_) {
        first_scan_ = false;

        // TODO is this the correct way to initialize? (need to have some laser scan to use for first comparison)
	odom_angle_at_last_laser_align_ = prev_odom_angle_;
        odom_loc_at_last_laser_align_ = prev_odom_loc_;
        convertRangesToPointCloud(ranges, angle_min, angle_max, range_max, most_recent_used_scan_);
        return;
    }

    if (!shouldProcessLaser()) {
        return;
    }

    ROS_INFO_STREAM("Curr odom " << prev_odom_loc_.x() << ", " << prev_odom_loc_.y() << ", " << prev_odom_angle_);
    ROS_INFO_STREAM("Last update " << odom_loc_at_last_laser_align_.x() << ", " << odom_loc_at_last_laser_align_.y() << ", " << odom_angle_at_last_laser_align_);

    if (use_gtsam_) {
        ObserveLaserMultipleScansCompared(ranges, range_min, range_max, angle_min, angle_max);
        return;
    }

    // Compute the odometry offset from the last recorded scan to use as an initial guess for the relative transform
    // between poses
    Eigen::Vector2f unrotated_odom_est_loc_displ = prev_odom_loc_ - odom_loc_at_last_laser_align_;
    float odom_est_angle_displ = math_util::AngleDiff(prev_odom_angle_, odom_angle_at_last_laser_align_);
    Eigen::Rotation2Df rotate(-1 * prev_odom_angle_);
    Vector2f odom_est_loc_displ = rotate * unrotated_odom_est_loc_displ;

    // Compute the point cloud relative to the robot's current position
    vector<Eigen::Vector2f> current_point_cloud;
    convertRangesToPointCloud(ranges, angle_min, angle_max, range_max, current_point_cloud);

    // Update the rasterized lookup. Will be relative to the pose of the robot at the last timestep
    updateRasterizedLookup(most_recent_used_scan_);

    // Compute log prob for possible poses
    vector<RelativePoseResults> relative_pose_results;
    computeLogProbsForPoseGrid(current_point_cloud, odom_est_loc_displ, odom_est_angle_displ, true, true, relative_pose_results);

    // Compute the maximum likelihood translation and rotation between the previous scan and this one
    Vector2f maximum_likelihood_scan_offset_position;
    float maximum_likelihood_scan_offset_angle;
    getMaximumLikelihoodScanOffset(relative_pose_results, true, maximum_likelihood_scan_offset_position,
                                   maximum_likelihood_scan_offset_angle);

    ROS_INFO_STREAM("Maximum likelihood offset " << maximum_likelihood_scan_offset_position.x() << ", " << maximum_likelihood_scan_offset_position.y() << ", " << maximum_likelihood_scan_offset_angle);
    ROS_INFO_STREAM("Odom offset " << odom_est_loc_displ.x() << ", " << odom_est_loc_displ.y() << ", " << odom_est_angle_displ);

    // Computing expected pose and covariance
    std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> poseAndCovariance;
    poseAndCovariance = computeMeanPoseAndCovariance(relative_pose_results);

    // Updating Trajectory Estimate
    updateTrajectoryEstimates(poseAndCovariance, std::make_pair(maximum_likelihood_scan_offset_position, maximum_likelihood_scan_offset_angle));

    // Update the fields for the next laser reading
    laser_observations_for_pose_in_trajectory_.emplace_back(current_point_cloud);
    odom_angle_at_last_laser_align_ = prev_odom_angle_;
    odom_loc_at_last_laser_align_ = prev_odom_loc_;
    most_recent_used_scan_ = current_point_cloud;
}

Eigen::Matrix3d SLAM::computeRelativeCovariance(const std::vector<RelativePoseResults> &rel_poses_with_likelihood){
     
    Eigen::Matrix3d K=Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix3d u=Eigen::MatrixXd::Zero(3,3);
    double cumulative_probability=0;
    for (const RelativePoseResults &pose : rel_poses_with_likelihood){
    	double obs_prob = std::exp(pose.obs_log_probability_);
	    Vector2f rel_loc = pose.location_offset_;
	    float rel_angle = pose.rotation_offset_;
	    Eigen::Matrix3d relativePose;
        relativePose << cos(rel_angle), -sin(rel_angle), rel_loc.x(),
                        sin(rel_angle),  cos(rel_angle), rel_loc.y(),
                        0,               0,              1;
	    // Update K
	    K += relativePose * relativePose.transpose() * obs_prob;
	    // Update u
	    u += relativePose * obs_prob;
	    if (cumulative_probability == 0) {
	        ROS_INFO_STREAM("Rel loc " << rel_loc);
	        ROS_INFO_STREAM("Rel angle " << rel_angle);
	        ROS_INFO_STREAM("obs prob " << obs_prob);
	        ROS_INFO_STREAM("Relative pose " << relativePose);
	    }
	    // cumulative prob
	    cumulative_probability += obs_prob;
    }

    ROS_INFO_STREAM("K " << K);
    ROS_INFO_STREAM("U " << u);
    ROS_INFO_STREAM("Cumulative prob: " << cumulative_probability);
    Eigen::Matrix3d Covariance;
    Covariance = K/cumulative_probability - u*u.transpose()/(std::pow(cumulative_probability, 2)); 
    return Covariance;    
}

std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> SLAM::computeMeanPoseAndCovariance(const std::vector<RelativePoseResults> &poses_with_likelihood){

    float est_angle_offset = 0.0;
    Eigen::Vector2f est_loc_offset(0,0);
    Eigen::Matrix3f K=Eigen::MatrixXf::Zero(3,3);
    Eigen::Matrix3f u=Eigen::MatrixXf::Zero(3,3);

    std::pair<Eigen::Vector2f, float> prevLocInfo;
    if (trajectory_estimates_.empty()) {
        prevLocInfo = std::make_pair(Vector2f(0, 0), 0);
    } else {
        prevLocInfo = trajectory_estimates_.back().first;
    }
    Eigen::Vector2f lastTimeStepLoc = prevLocInfo.first;
    float lastTimeStepAngle = prevLocInfo.second;

    double cumulativeTotalProbability=0.;
    for (const RelativePoseResults &pose : poses_with_likelihood){
        double totalProb = std::exp(pose.obs_log_probability_ + pose.motion_log_probability_);

        est_loc_offset += pose.location_offset_ * totalProb;
        est_angle_offset += pose.rotation_offset_*totalProb;
        cumulativeTotalProbability += totalProb;

        float globalAngle = pose.rotation_offset_+lastTimeStepAngle;
        Eigen::Vector2f globalLoc = pose.location_offset_+lastTimeStepLoc;
        Eigen::Matrix3f globalPose;
        globalPose << cos(globalAngle), -sin(globalAngle), globalLoc.x(),
        sin(globalAngle),  cos(globalAngle), globalLoc.y(),
        0,         0,            1;

        K += globalPose * globalPose.transpose() * totalProb;
        u += globalPose * totalProb;
    }

    // Mean Pose Estimate
    est_loc_offset = est_loc_offset/cumulativeTotalProbability;
    est_angle_offset = est_angle_offset/cumulativeTotalProbability;

    // Covariance
    Eigen::Matrix3f Covariance;
    Covariance = (K / cumulativeTotalProbability) - ((u * u.transpose()) / (std::pow(cumulativeTotalProbability, 2)));

    return std::make_pair(std::make_pair(est_loc_offset, est_angle_offset), Covariance);
}

void SLAM::updateTrajectoryEstimates(
        const std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> &nextBestPoseAndCov,
        const std::pair<Eigen::Vector2f, float> &MLEPoseAndAngleOffset) {

    std::pair<Eigen::Vector2f, float> prevLocInfo;
    if (trajectory_estimates_.empty()) {
        prevLocInfo = std::make_pair(Vector2f(0, 0), 0.0);
    } else {
       prevLocInfo = trajectory_estimates_.back().first;
    }
    Eigen::Vector2f lastTimeStepLoc = prevLocInfo.first;
    float lastTimeStepAngle = prevLocInfo.second;
    Rotation2Df eig_rotation(lastTimeStepAngle);
    Vector2f rotated_offset = eig_rotation * MLEPoseAndAngleOffset.first;

    Eigen::Vector2f latestTrajPointLoc = lastTimeStepLoc + rotated_offset;
    float latestTrajPointAngle = lastTimeStepAngle + MLEPoseAndAngleOffset.second;
    Eigen::Matrix3f latestTrajPointCov = nextBestPoseAndCov.second;
    trajectory_estimates_.emplace_back(std::make_pair(
            std::make_pair(latestTrajPointLoc, latestTrajPointAngle), latestTrajPointCov));
    odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
}


void SLAM::updateRasterizedLookup(const vector<Vector2f> &reference_scan) {
    for (int row = 0; row < raster_rows_and_cols_; row++) {
        for (int column = 0; column < raster_rows_and_cols_; column++) {
            double max_log_prob = -std::numeric_limits<double>::infinity();
            float raster_cell_center_x = (column * raster_grid_increment_) + raster_grid_center_offset_;
            float raster_cell_center_y = (row * raster_grid_increment_) + raster_grid_center_offset_;
            Vector2f raster_cell_center(raster_cell_center_x, raster_cell_center_y);

            for (const Vector2f &scan_point : reference_scan) {
                double log_prob_for_point = -((scan_point - raster_cell_center).squaredNorm()) / (2 * laser_variance_);
                max_log_prob = std::max(log_prob_for_point, max_log_prob); // TODO sum or max?
            }
            raster_mat_with_log_probs_(row, column) = max_log_prob;
        }
    }

    publishRasterAsImage();
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
    if (!odom_initialized_) {
        prev_odom_angle_ = odom_angle;
        prev_odom_loc_ = odom_loc;
        odom_initialized_ = true;
        return;
    }

    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;

  ROS_INFO_STREAM("Trajectory estiamtes size " << trajectory_estimates_.size());
  ROS_INFO_STREAM("scans size " << laser_observations_for_pose_in_trajectory_.size());

  for (size_t i = 0; i < laser_observations_for_pose_in_trajectory_.size(); i++) {
      float pose_angle = trajectory_estimates_[i].first.second;
      Vector2f loc = trajectory_estimates_[i].first.first;
      Eigen::Matrix3f pose_mat;
      pose_mat << cos(pose_angle), -sin(pose_angle), loc.x(),
      sin(pose_angle),  cos(pose_angle), loc.y(),
      0,         0,            1;

      vector<Vector2f> scan_at_pose = laser_observations_for_pose_in_trajectory_[i];
      for (const Vector2f &scan_point : scan_at_pose) {
          Eigen::Vector3f extra_entry_scan_point(scan_point.x(), scan_point.y(), 1);
          Eigen::Vector3f extra_entry_transformed_point = pose_mat * extra_entry_scan_point;
          Vector2f transformed_point(extra_entry_transformed_point.x(), extra_entry_transformed_point.y());

          map.emplace_back(transformed_point);
      }
  }
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam

