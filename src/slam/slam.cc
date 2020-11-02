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

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM(ros::NodeHandle *node_handle) :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    node_handle_(node_handle) {

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

    raster_grid_center_offset_ = (0.5 * raster_grid_increment_) - (0.5 * raster_grid_size_);

    raster_rows_and_cols_ = ceil(raster_grid_size_ / raster_grid_increment_);
    raster_mat_with_log_probs_.resize(raster_rows_and_cols_, raster_rows_and_cols_);

    image_pub_ =
            node_handle_->advertise<sensor_msgs::Image>("raster_img", 1);
}

void SLAM::publishTrajectory(amrl_msgs::VisualizationMsg &vis_msg) {
    for (const auto &trajectory_point_with_cov : trajectory_estimates_) {
        std::pair<Vector2f, float> trajectory_point = trajectory_point_with_cov.first;
        float line_seg_len = 0.4;
        Vector2f secondpoint(line_seg_len * cos(trajectory_point.second), line_seg_len * sin(trajectory_point.second));
        uint32_t trajectory_color = 0x34b4eb;
        visualization::DrawLine(trajectory_point.first,
                                trajectory_point.first + secondpoint,
                                trajectory_color, vis_msg);
    }

    for (const auto &odom_pos : odom_only_estimates_) {
        float line_seg_len = 0.4;
        Vector2f secondpoint(line_seg_len * cos(odom_pos.second), line_seg_len * sin(odom_pos.second));
        uint32_t trajectory_color = 0x3400eb;
        visualization::DrawLine(odom_pos.first,
                                odom_pos.first + secondpoint,
                                trajectory_color, vis_msg);
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
    
    // Return the latest pose estimate of the robot.
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
    image.is_bigendian = false; // TODO?
    image.height = raster_rows_and_cols_;
    image.width = raster_rows_and_cols_;
    image.step = raster_rows_and_cols_;

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

    for (int y = 0; y < raster_rows_and_cols_; y++) {
        for (int x = 0; x < raster_rows_and_cols_; x++) {
            double raster_val = raster_mat_with_log_probs_(y, x);
            double scaled_double = ((std::exp(raster_val) - std::exp(min_raster)) / (std::exp(max_raster) - std::exp(min_raster))) * 255.0;
            uint8_t scaled = scaled_double;
            image.data.emplace_back(scaled);
        }
    }

    ROS_INFO_STREAM("Publishing image");
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

        if ((lookup_row >= 0) && (lookup_row < raster_rows_and_cols_)
                && (lookup_column >= 0) && (lookup_column < raster_rows_and_cols_)) {
            log_prob += raster_mat_with_log_probs_(lookup_row, lookup_column);
        }
        // TODO how should we deal with points that aren't in the raster range? Just ignore them?
    }
    return log_prob;
}

double SLAM::computeMotionLogProbForRelativePose(const Eigen::Vector2f &position_offset,  const float &angle_offset,
						 const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset){
	
	double motionLogProb;
	
	// These three values can be pre-calculated if things seem to be running too slow.
	//float delta_rot_1 = atan2(odom_position_offset.y(), odom_position_offset.x()) - odom_angle_at_last_laser_align_;
    	float delta_rot_1 = atan2(odom_position_offset.y(), odom_position_offset.x());
	float delta_trans = odom_position_offset.norm();
    	float delta_rot_2 = odom_angle_offset - delta_rot_1;

    std::pair<Vector2f, float> prev_loc_info;
    if (trajectory_estimates_.empty()) {
        prev_loc_info = std::make_pair(Vector2f(0, 0), 0.0);
    } else {
        prev_loc_info = trajectory_estimates_.back().first;
    }
	//Eigen::Vector2f lastTimeStepLoc = prevLocInfo.first.first;
    	//float lastTimeStepAngle = prev_loc_info.second;
	
	//Eigen::Vector2f possibleCurrLoc = position_offset + lastTimeStepLoc;
	//float possibleCurrAngle = angle_offset + lastTimeStepAngle;
	
	//float delta_rot_1_hat = atan2(position_offset.y(), position_offset.x()) - lastTimeStepAngle;  
	float delta_rot_1_hat = atan2(position_offset.y(), position_offset.x());
	float delta_trans_hat = position_offset.norm();
	float delta_rot_2_hat =	angle_offset - delta_rot_1_hat;
	
	float delta_rot_1_std_dev = motion_model_rot_error_from_rot_*delta_rot_1_hat + motion_model_rot_error_from_transl_*delta_trans_hat;
	float delta_trans_std_dev = motion_model_transl_error_from_rot_*(delta_rot_1_hat+delta_rot_2_hat) + motion_model_transl_error_from_transl_*delta_trans_hat;
	float delta_rot_2_std_dev = motion_model_rot_error_from_rot_*delta_rot_2_hat + motion_model_rot_error_from_transl_*delta_trans_hat; 

	motionLogProb  = -std::pow(delta_rot_1-delta_rot_1_hat, 2)/(2*std::pow(delta_rot_1_std_dev, 2));
	motionLogProb += -std::pow(delta_trans-delta_trans_hat ,2)/(2*std::pow(delta_trans_std_dev ,2));
	motionLogProb += -std::pow(delta_rot_2-delta_rot_2_hat ,2)/(2*std::pow(delta_rot_2_std_dev ,2));

	return motionLogProb;
}

void SLAM::computeLogProbsForRotatedScans(const vector<Vector2f> &rotated_current_scan, const float &angle,
                                          const vector<float> &possible_x_offsets,
                                          const vector<float> &possible_y_offsets,
					  const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset,
                                          vector<RelativePoseResults> &log_prob_results) {

    // Scan will have already been rotated by the time this is called, so we just have to translate the points
    for (const float &x_offset : possible_x_offsets) {
        for (const float &y_offset : possible_y_offsets) {
            Vector2f position_offset(x_offset, y_offset);
            double log_prob = computeLogProbForRelativePose(rotated_current_scan, position_offset);
	        double motion_log_prob =  computeMotionLogProbForRelativePose(position_offset, angle, odom_position_offset, odom_angle_offset);
	        RelativePoseResults result;
            result.location_offset_ = position_offset;
            result.rotation_offset_ = angle;
            result.obs_log_probability_ = log_prob;
	        result.motion_log_probability_ = motion_log_prob;
            log_prob_results.emplace_back(result);
        }
    }
}

void SLAM::computeLogProbsForPoseGrid(const vector<Vector2f> &current_scan, const Vector2f &odom_position_offset,
                                      const float &odom_angle_offset,
                                      vector<RelativePoseResults> &relative_pose_results) {

    // Compute the standard deviation based on the translation and rotation since the last laser observation
    float transl_std_dev = (motion_model_transl_error_from_transl_ * (odom_position_offset.norm())) +
            (motion_model_transl_error_from_rot_ * fabs(odom_angle_offset));
    float rot_std_dev = (motion_model_rot_error_from_transl_ * (odom_position_offset.norm())) +
            (motion_model_rot_error_from_rot_ * fabs(odom_angle_offset));
	
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

    relative_pose_results.clear();
    size_t reserve_size = possible_rotations.size() * possible_y_offsets.size() * possible_x_offsets.size();
    relative_pose_results.reserve(reserve_size);

    // Iterate over the angles in the outermost loop so that we can rotate the scans only once for each angle
    for (const float &rot_angle : possible_rotations) {
//        ROS_INFO_STREAM("Iterating for rotation " << rot_angle);
        Rotation2Df eig_rotation(rot_angle); // TODO does this need to be negated?
        vector<Vector2f> rotated_scan;
        rotated_scan.reserve(current_scan.size());
        for (const Vector2f &point : current_scan) {
            rotated_scan.emplace_back(eig_rotation * point); // TODO verify
        }
//        ROS_INFO_STREAM("Rotated scan");
        computeLogProbsForRotatedScans(rotated_scan, rot_angle, possible_x_offsets, possible_y_offsets,
                                       odom_position_offset, odom_angle_offset, relative_pose_results);
//        ROS_INFO_STREAM("Computed log probs");
    }
}

void SLAM::getMaximumLikelihoodScanOffset(const vector<RelativePoseResults> &relative_pose_results,
                                    Vector2f &max_likelihood_position_offset, float &max_likelihood_angular_offset) {
    double max_likelihood = -std::numeric_limits<double>::infinity();
    double totalLogLikelihood;
    for (const RelativePoseResults &pose_result : relative_pose_results) {
        totalLogLikelihood = pose_result.motion_log_probability_+pose_result.obs_log_probability_;
//        totalLogLikelihood = pose_result.obs_log_probability_;
	if (totalLogLikelihood > max_likelihood) {
            max_likelihood_position_offset = pose_result.location_offset_;
            max_likelihood_angular_offset = pose_result.rotation_offset_;
            max_likelihood = totalLogLikelihood;
//            ROS_INFO_STREAM("Found new ML solution " << max_likelihood_position_offset.x() << ", " << max_likelihood_position_offset.y());
        }
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

        // TODO is this the correct way to initialize? (need to have some laser scan to use for first comparison)
        //odom_angle_at_last_laser_align_ = prev_odom_angle_;
        //odom_loc_at_last_laser_align_ = prev_odom_loc_;
        //convertRangesToPointCloud(ranges, angle_min, angle_max, range_max, most_recent_used_scan_);
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
    computeLogProbsForPoseGrid(current_point_cloud, odom_est_loc_displ, odom_est_angle_displ, relative_pose_results);

    // Compute the maximum likelihood translation and rotation between the previous scan and this one
    Vector2f maximum_likelihood_scan_offset_position;
    float maximum_likelihood_scan_offset_angle;
    getMaximumLikelihoodScanOffset(relative_pose_results, maximum_likelihood_scan_offset_position,
                                   maximum_likelihood_scan_offset_angle);
//    maximum_likelihood_scan_offset_position = odom_est_loc_displ;
//    maximum_likelihood_scan_offset_angle = odom_est_angle_displ;

    ROS_INFO_STREAM("Maximum likelihood offset " << maximum_likelihood_scan_offset_position.x() << ", " << maximum_likelihood_scan_offset_position.y() << ", " << maximum_likelihood_scan_offset_angle);
    ROS_INFO_STREAM("Odom offset " << odom_est_loc_displ.x() << ", " << odom_est_loc_displ.y() << ", " << odom_est_angle_displ);

    // Computing expected pose and covariance
    std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> poseAndCovariance;
//    ROS_INFO_STREAM("Computing cov");
    poseAndCovariance = computeMeanPoseAndCovariance(relative_pose_results);
//    ROS_INFO_STREAM("Update trajectory estimates");
    // Updating Trajectory Estimate
    updateTrajectoryEstimates(poseAndCovariance, std::make_pair(maximum_likelihood_scan_offset_position, maximum_likelihood_scan_offset_angle));

//    ROS_INFO_STREAM("Done updating trajectory estimates");

    // Update the fields for the next laser reading
    laser_observations_for_pose_in_trajectory_.emplace_back(current_point_cloud);
    odom_angle_at_last_laser_align_ = prev_odom_angle_;
    odom_loc_at_last_laser_align_ = prev_odom_loc_;
    most_recent_used_scan_ = current_point_cloud;
}

std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> SLAM::computeMeanPoseAndCovariance(const std::vector<RelativePoseResults> &poses_with_likelihood){

    float est_angle_offset=0.;
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
	
	est_loc_offset += pose.location_offset_*totalProb;
	est_angle_offset += pose.rotation_offset_*totalProb;
	cumulativeTotalProbability += totalProb;
	
	float globalAngle = pose.rotation_offset_+lastTimeStepAngle;
       	Eigen::Vector2f globalLoc = pose.location_offset_+lastTimeStepLoc;	
	Eigen::Matrix3f globalPose;
        globalPose << cos(globalAngle), -sin(globalAngle), globalLoc.x(),
		      sin(globalAngle),  cos(globalAngle), globalLoc.y(),
		      0,		 0, 		   1;

	K += globalPose * globalPose.transpose() * totalProb;
	u += globalPose * totalProb;
    }
    // Mean Pose Estimate
    est_loc_offset =  est_loc_offset/cumulativeTotalProbability;
    est_angle_offset = est_angle_offset/cumulativeTotalProbability;
    // Covariance
    Eigen::Matrix3f Covariance;
    Covariance = K/cumulativeTotalProbability - u*u.transpose()/(std::pow(cumulativeTotalProbability, 2));

    return std::make_pair(std::make_pair(est_loc_offset, est_angle_offset), Covariance);
}

void SLAM::updateTrajectoryEstimates(const std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> &nextBestPoseAndCov,
				     const std::pair<Eigen::Vector2f, float> &MLEPoseAndAngleOffset){

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
    trajectory_estimates_.push_back(std::make_pair(std::make_pair(latestTrajPointLoc, latestTrajPointAngle), latestTrajPointCov));
    odom_only_estimates_.push_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
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
//    odom_loc_at_last_laser_align_ = prev_odom_loc_;
//    odom_angle_at_last_laser_align_ = prev_odom_angle_;
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
              0,		 0, 		   1;

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

