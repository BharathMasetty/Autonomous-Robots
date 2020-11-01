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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>
#include <amrl_msgs/VisualizationMsg.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

    /**
     * Results for evaluation of a relative pose between scans.
     */
    struct RelativePoseResults {

        /**
         * Translation of robot pose from scan i to scan i+1.
         */
        Eigen::Vector2f location_offset_;

        /**
         * Rotation of robot pose from scan i to scan i+1.
         */
        float rotation_offset_;

        /**
         * Log probability of the relative pose.
         */
        double obs_log_probability_;

	/**
	 * Motion model log probability of the relatve pose
	 */
	double motion_log_probability_;
    };

class SLAM {
 public:
  // Default Constructor.
  SLAM(ros::NodeHandle *node_handle);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  void publishTrajectory(amrl_msgs::VisualizationMsg &visualization_msg);

 private:

    /**
     * Offset of the laser from the base_link frame in the x direction.
     */
    const float kBaselinkToLaserOffsetX = 0.2;

    /**
     * ROS Parameter name for the minimum change in the position reported by odometry between two evaluated laser scans.
     */
    const std::string kLaserUpdateOdomLocDifferenceParamName = "laser_update_odom_loc_diff";

    /**
     * ROS Parameter name for the minimum change in rotation reported by odometry between two evaluated laser scans.
     */
    const std::string kLaserUpdateOdomAngleDifferenceParamName = "laser_update_odom_rot_diff";

    /**
     * Default value for the minimum change in the position reported by odometry between two evaluated laser scans.
     *
     * TODO tune this value.
     */
    const float kDefaultLaserUpdateOdomLocDifference = 0.5;

    /**
     * Default value for the minimum change in rotation reported by odometry between two evaluated laser scans.
     *
     * TODO tune this value.
     */
    const float kDefaultLaserUpdateOdomAngleDifference = math_util::DegToRad(30.0);

    /**
     * ROS parameter name for the laser variance (squared std dev).
     */
    const std::string kLaserVarianceParamName = "laser_variance";

    /**
     * Default value for the laser variance (squared std dev).
     *
     * TODO tune this value.
     */
    const float kDefaultLaserVariance = 0.05;

    /**
     * ROS parameter name for the resolution of the raster table.
     */
    const std::string kRasterGridIncrementParamName = "raster_grid_increment";

    /**
     * ROS Parameter name for the size of one side of the square containing raster results in meters. This should be a
     * multiple of the raster grid increment.
     */
    const std::string kRasterGridSizeParamName = "raster_grid_size";

    /**
     * Default value for the resolution of the raster table.
     *
     * TODO tune this value.
     */
    const float kDefaultRasterGridIncrement = 0.01;

    /**
     * Default value for the size of one side of the square containing raster results in meters. This should be a
     * multiple of the raster grid increment.
     *
     * TODO tune this value.
     */
    const float kDefaultRasterGridSize = 5.0;

    /**
     * ROS Parameter name for the resolution of the pose search cube in the x and y dimensions.
     */
    const std::string kPoseEvalTranslIncrementParamName = "pose_eval_transl_inc";

    /**
     * ROS Parameter name for the resolution of the pose search cube in the rotation dimension.
     */
    const std::string kPoseEvalRotIncrementParamName = "pose_eval_rot_inc";

    /**
     * Default value for the resolution of the pose search cube in the x and y dimensions.
     *
     * Ex. if this is set to 0.05, x's within the desired confidence interval will be evaluated at every 0.05 m.
     *
     * TODO tune this
     */
    const float kDefaultPoseEvalTranslIncrement = 0.005;

    /**
     * Default value for the resolution of the pose search cube in the rotation dimension.
     *
     * Ex. if this is set to 0.1, theta values within the desired confidence interval will be evaluated at every 0.1
     * radians.
     *
     * TODO tune this
     */
    const float kDefaultPoseEvalRotIncrement = math_util::DegToRad(0.5);

    /**
     * ROS Parameter name for how many standard deviations (on each side of odom pose) we should compute likelihoods
     * for poses for.
     */
    const std::string kPoseSearchStdDevMultiplierParamName = "pose_search_std_dev_multiplier";

    /**
     * Default value to use for how many standard deviations (on each side of odom pose) we should compute likelihoods
     * for poses for.
     *
     * TODO tune this
     */
    const float kDefaultPoseSearchStdDevMultiplier = 2.0;

    /**
     * ROS Parameter names for the motion model std dev scale factors.
     */
    const std::string kMotionModelTranslErrorFromRotParamName = "motion_model_transl_from_rot";
    const std::string kMotionModelTranslErrorFromTranslParamName = "motion_model_transl_from_transl";
    const std::string kMotionModelRotErrorFromRotParamName = "motion_model_rot_from_rot";
    const std::string kMotionModelRotErrorFromTranslParamName = "motion_model_rot_from_transl";

    /**
     * Default motion model std dev scale factors.
     *
     * TODO tune these
     */
    const float kDefaultMotionModelTranslErrorFromRot = 0.1;
    const float kDefaultMotionModelTranslErrorFromTransl = 0.15;
    const float kDefaultMotionModelRotErrorFromRot = 0.15;
    const float kDefaultMotionModelRotErrorFromTransl = 0.15;

    bool first_scan_ = true;

    /**
     * Laser variance (squared std dev).
     */
    float laser_variance_;

    /**
     * Minimum change in the position reported by odometry between two evaluated laser scans.
     */
    float laser_update_odom_loc_difference_;

    /**
     * Minimum change in rotation reported by odometry between two evaluated laser scans.
     */
    float laser_update_odom_angle_difference_;

    /**
     * Resolution of the raster table.
     */
    float raster_grid_increment_;

    /**
     * Size of one side of the square containing raster results in meters. This should be a
     * multiple of the raster grid increment.
     */
    float raster_grid_size_;

    /**
     * raster_grid_center_offset_ is the location of the bottom/leftmost edges of the raster grid that makes the center
     * of the grid happen at (0, 0).
     */
    float raster_grid_center_offset_;

    /**
     * Resolution of the pose search cube in the x and y dimensions.
     *
     * Ex. if this is set to 0.05, x's within the desired confidence interval will be evaluated at every 0.05 m.
     */
    float pose_eval_transl_increment_;

    /**
     * Resolution of the pose search cube in the rotation dimension.
     *
     * Ex. if this is set to 0.1, theta values within the desired confidence interval will be evaluated at every 0.1
     * radians.
     */
    float pose_eval_rot_increment_;

    /**
     * Multiplier for the pose standard deviation that can be used to change the confidence interval.
     *
     * Ex. to search +/- 2 standard deviations around the odometry estimated pose, this should be set to 2.
     */
    float pose_search_std_dev_multiplier_;

    /**
     * Motion model std dev scale factors.
     *
     * Will provide the standard deviation given given 0 rotation and 1 meter of translation and 1 radian of rotation
     * and 0 meters of translation (for the rot and transl components, respectively)
     */
    float motion_model_transl_error_from_rot_;
    float motion_model_transl_error_from_transl_;
    float motion_model_rot_error_from_rot_;
    float motion_model_rot_error_from_transl_;
	
    //Translation and Rotation Standard deviation
    float trans_std_dev_squared_;
    float rot_std_dev_squared_;

    // Previous odometry-reported locations.
    Eigen::Vector2f prev_odom_loc_;
    float prev_odom_angle_;
    bool odom_initialized_;

    /**
     * Node handle.
     */
    ros::NodeHandle *node_handle_;

    /**
     * Odometry position at the last laser scan used.
     */
    Eigen::Vector2f odom_loc_at_last_laser_align_;

    /**
     * Odometry angle at the last laser scan used.
     */
    float odom_angle_at_last_laser_align_;

    /**
     * Number of rows and columns in the rasterization.
     */
    int raster_rows_and_cols_;

    /**
     * Rasterized lookup table that contains the probability of observing a point at the given position.
     *
     * This should only be used by ObserveLaser.
     */
    Eigen::MatrixXd raster_mat_with_log_probs_;

    void publishRasterAsImage();

    /**
     * Laser observations for each pose in the robot trajectory.
     */
    std::vector<std::vector<Eigen::Vector2f>> laser_observations_for_pose_in_trajectory_;

    /**
     * Robot's estimated trajectory. TODO uncomment when this is actually used.
     */
    // std::vector<std::pair<Eigen::Vector2f, float>> robot_trajectory_;

    /**
     * Most recently considered scan (taken at the last entry in robot_trajectory_).
     */
    std::vector<Eigen::Vector2f> most_recent_used_scan_;

    /**
     * Trajectory Estimates with respect to odom frame
     */
    std::vector<std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f>> trajectory_estimates_;

    std::vector<std::pair<Eigen::Vector2f, float>> odom_only_estimates_;

    /**
     * Determine if the robot has moved far enough that we should compare the last used laser scan to the current laser
     * scan.
     *
     * @return True if the robot has moved far enough that we should compare the last used laser scan to the current
     * laser scan, false if we should skip this laser scan.
     */
    bool shouldProcessLaser();

    /**
     * Convert the laser scan ranges to a point cloud (x,y points relative to the base_link of the robot).
     *
     * @param ranges[in]        Range readings for the laser scan.
     * @param angle_min[in]     Minimum angle for the laser scan.
     * @param angle_max[in]     Maximum angle for the laser scan.
     * @param range_max[in]     Maximum range for the laser scanner. Points with this value don't reflect actual
     *                          objects, and consequently, should be filtered out of the scan used for map
     *                          construction/scan comparison.
     * @param point_cloud[out]  Point cloud (x,y points relative to the base_link of the robot)
     */
    void convertRangesToPointCloud(const std::vector<float>& ranges, const float &angle_min, const float &angle_max,
                                   const float &range_max, std::vector<Eigen::Vector2f> &point_cloud);

    /**
     * Update the variable containing the rasterized version of the reference scan.
     *
     * Rasterized version gives the log probabilities of observations at points relative to the base_link of the robot
     * when it made the reference scan.
     *
     * @param reference_scan Reference scan to use in generating the rasterized version.
     */
    void updateRasterizedLookup(const std::vector<Eigen::Vector2f> &reference_scan);

    ros::Publisher image_pub_;

    /**
     * Compute the maximum likelihood scan-scan transform given the results from the pose evaluation.
     *
     * @param relative_pose_results[in]             Results from the pose evaluation based on the current scan and the
     *                                              rasterized lookup table based on the previous scan.
     * @param max_likelihood_position_offset[out]   Position offset that had the maximum likelihood.
     * @param max_likelihood_angular_offset[out]    Rotation offset that hadn the maximum likelihood.
     */
    void getMaximumLikelihoodScanOffset(const std::vector<RelativePoseResults> &relative_pose_results,
                                        Eigen::Vector2f &max_likelihood_position_offset,
                                        float &max_likelihood_angular_offset);

    /**
     * Compute log probabilities for a discretized grid of poses centered around the odometry estimate for the robot
     * relative to the base_link pose at the rasterized scan.
     *
     * @param current_scan[in]              Scan at the robot's current position.
     * @param odom_position_offset[in]      Position difference since last scan estimated by odometry (should search
     *                                      centered around this position).
     * @param odom_angle_offset[in]         Angular difference since last scan estimated by odometry (should search
     *                                      centered around this rotation).
     * @param relative_pose_results[out]    Results containing each evaluated transform (relative position and rotation
     *                                      from the robot pose at the previous scan to robot pose at the current scan)
     *                                      and the log likelihood of the evaluated transform.
     */
    void computeLogProbsForPoseGrid(const std::vector<Eigen::Vector2f> &current_scan,
                                    const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset,
                                    std::vector<RelativePoseResults> &relative_pose_results);

    /**
     * Compute the log probability for each possible translation given by the possible x and y offsets based on
     * alignment between the rasterized scan and the rotated scan centered at relative to the robot's current position.
     *
     * @param rotated_current_scan[in]      Rotated laser scan relative to the robot's current position.
     * @param angle[in]                     Angle that the scan was rotated. This is only used to store the results,
     *                                      since the scan was already rotated.
     * @param possible_x_offsets[in]        Possible X components of the relative pose to evaluate.
     * @param possible_y_offsets[in]        Possible Y components of the relative pose to evaluate
     * @param odom_position_offset[in]      Position difference since last scan estimated by odometry (should search
     *                                      centered around this position).
     * @param odom_angle_offset[in]         Angular difference since last scan estimated by odometry (should search
     *                                      centered around this rotation).

     * @param log_prob_results[out]         Results from the scan comparison for each of the points. This will be
     *                                      aggregated across several calls to this function with different rotations, so
     *                                      this value should only be added to (not cleared or deleted).
     */
    void computeLogProbsForRotatedScans(const std::vector<Eigen::Vector2f> &rotated_current_scan, const float &angle,
                                        const std::vector<float> &possible_x_offsets,
                                        const std::vector<float> &possible_y_offsets,
					const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset,
                                        std::vector<RelativePoseResults> &log_prob_results);

    /**
     * Compute the log probability for the given relative pose (scan already rotated).
     *
     * @param rotated_current_scan  Rotated scan at current robot pose.
     * @param position_offset       Position offset between robot pose for raster and current robot pose that should be
     *                              evaluated.
     *
     *
     * @return Log probability of the position offset between the robot pose for the raster and the current robot pose
     * (after the scan was rotated).
     */
    double computeLogProbForRelativePose(const std::vector<Eigen::Vector2f> &rotated_current_scan,
                                         const Eigen::Vector2f &position_offset);
    /**
     * Compute the Motion Model likelihood for a given relative pose
     * @param position_offset 	    	    Position offset between robot pose for raster and current robot pose that should be evaluated.
     * 
     * @param angle_offset  	    	    Angle offset between robot pose for raster and current robot pose that should be evaluated.
     *
     * @param odom_position_offset[in]      Position difference since last scan estimated by odometry (should search
     *                                      centered around this position).
     * 
     * @param odom_angle_offset[in]         Angular difference since last scan estimated by odometry (should search
     *                                      centered around this rotation).
     *
     */
    double computeMotionLogProbForRelativePose(const Eigen::Vector2f &position_offset,  const float &angle_offset,
		    				const Eigen::Vector2f &odom_position_offset, const float &odom_angle_offset);

    /**
     * Compute the estimate of the pose and covariance as described in CSM paper
     * 
     * @param poses_with_likelihood Vector of RelativePoseResults with computed probabilities
     *
     * @return ((E(loc), E(angle)), Covariance)
     */
    std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> computeMeanPoseAndCovariance(const std::vector<RelativePoseResults> &poses_with_likelihood);
    
    /**
     * Add the latest MLE estimate of the robot location and angle with respect to odom origin to trajectory_estimates_ vector
     *
     * @param ((E(loc), E(angle)), Covariance)
     * @param (MLE(loc), MLE(anlge))
     */
    void updateTrajectoryEstimates(const std::pair<std::pair<Eigen::Vector2f, float>, Eigen::Matrix3f> &nextBestPoseAndCov, const std::pair<Eigen::Vector2f, float> &MLEPoseAndAngleOffset);
};
}  // namespace slam

#endif   // SRC_SLAM_H_

