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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

double Navigation::computeDecelDistance(const double &velocity_to_decelerate_from) {
    return std::pow(velocity_to_decelerate_from, 2) / std::abs(kMaxDecel);
}

std::vector<double> Navigation::getCurvaturesToEvaluate() {

    // TODO: Amanda, implement this
    return {};
}

std::pair<double, double> Navigation::chooseCurvatureForNextTimestep(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map) {

    // TODO: Amanda, implement this
    double best_curvature = 0;
    double distance_for_curvature = 0;

    return std::make_pair(best_curvature, distance_for_curvature);
}

std::unordered_map<double, std::pair<double, double>> Navigation::getFreePathLengthsAndClearances(
        const std::vector<double> &curvatures_to_evaluate) {
    // TODO: Bharath, feel free to replace this implementation if you can get the results more efficiently as a batch
    //  process. If you do, remove getFreePathLengthAndClearance.

    std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature;
    for (const double &curvature : curvatures_to_evaluate) {
        free_path_len_and_clearance_by_curvature[curvature] = getFreePathLengthAndClearance(curvature);
    }
    return free_path_len_and_clearance_by_curvature;
}

std::pair<double, double> Navigation::getFreePathLengthAndClearance(const double &curvature) {

    // TODO: Bharath, implement this or replace the implementation in getFreePathLengthsAndClearances and do it in
    //  batch.

    double free_path_len = 0;
    double clearance = 0;

    return std::make_pair(free_path_len, clearance);
}

void Navigation::executeTimeOptimalControl(const double &distance, const double &curvature) {
    // TODO: Kunal implement this.

    // This will probably have to be a bit different since we need to add latency compensation, but I've copied and
    // commented out the code from assignment 0 for reference.
    // if (computeDecelDistance(current_velocity_) >= distance) {
    //   // Decelerate
    //   current_velocity_ = std::max(0.0, kLoopExecutionDelay * kMaxDecel + current_velocity_);
    // } else {
    //   if (current_velocity_ < kMaxVel) {
    //     // Accelerate
    //     current_velocity_ = std::min(kMaxVel, kMaxAccel * kLoopExecutionDelay + current_velocity_);
    //   }
    //   // If not accelerating, we'll keep the same velocity as before
    // }

    // AckermannCurvatureDriveMsg drive_msg;
    // drive_msg.velocity = current_velocity_;
    // drive_pub_.publish(drive_msg);
}

void Navigation::Run() {

  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.

  if (drive_pub_.getNumSubscribers() == 0) {
    ROS_ERROR("Still no subscribers to Drive message. Not yet sending velocities.");
    return;
  }

  std::vector<double> curvatures_to_evaluate = getCurvaturesToEvaluate();
  std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature =
          getFreePathLengthsAndClearances(curvatures_to_evaluate);

  // TODO: (Amanda) may need to supply some notion of goal to this function.
  std::pair<double, double> curvature_and_dist_to_execute =
          chooseCurvatureForNextTimestep(free_path_len_and_clearance_by_curvature);
  executeTimeOptimalControl(curvature_and_dist_to_execute.second, curvature_and_dist_to_execute.first);
}

}  // namespace navigation
