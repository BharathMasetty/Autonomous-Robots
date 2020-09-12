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
    curvatures_to_evaluate_(constructCurvaturesToEvaluate()),
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
  n->param(kOpenClearanceThresholdParamName, open_clearance_threshold_, kDefaultDesiredClearance);
  double open_path_buffer_len;
  n->param(kOpenFreePathAfterStopDistThresholdParamName, open_path_buffer_len, kDefaultFreePathBufferLenThreshold);
  open_free_path_len_threshold_ = open_path_buffer_len + kApproxMaxVelStoppingDist;

  n->param(kScoringClearanceWeightParamName, scoring_clearance_weight_, kDefaultClearanceWeight);
  n->param(kScoringCurvatureWeightParamName, scoring_curvature_weight_, kDefaultCurvatureWeight);

  ROS_INFO_STREAM("Open path len threshold " << open_free_path_len_threshold_);
  ROS_INFO_STREAM("Open path clearance threshold " << open_clearance_threshold_);
  ROS_INFO_STREAM("Scoring clearance weight " << scoring_clearance_weight_);
  ROS_INFO_STREAM("Scoring curvature weight " << scoring_curvature_weight_);

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

std::vector<double> Navigation::constructCurvaturesToEvaluate() {
    int half_num_to_eval = (kNumCurvaturesToEval - 1) / 2;

    std::vector<double> eval_curves;
    eval_curves.resize(kNumCurvaturesToEval);

    // Calculating curves working our way out from 0 curvature instead of just coming up with a value to increment the
    // curvature by each time to ensure that exactly 0 is an option and so that our curvature options are symmetrical
    eval_curves[half_num_to_eval] = 0.0;
    for (int i = 1; i <= half_num_to_eval; i++) {
        int pos_curve_index = half_num_to_eval + i;
        int neg_curve_index = half_num_to_eval - i;
        double abs_curv = (((double) i ) / half_num_to_eval) * kMaxCurvature;
        eval_curves[pos_curve_index] = abs_curv;
        eval_curves[neg_curve_index] = -1 * abs_curv;
    }
    return eval_curves;
}

std::vector<double> Navigation::getCurvaturesToEvaluate() {
    return curvatures_to_evaluate_;
}

double Navigation::getFreePathLengthToClosestPointOfApproach(double goal_in_bl_frame_x, double curvature,
                                                             double obstacle_free_path_len) {
    if (curvature == 0.0) {
        return obstacle_free_path_len;
    }

    double rad_of_turn = 1.0 / abs(curvature);

    // This is the angle between the line from the center of turning to the goal position
    double arc_angle = atan(goal_in_bl_frame_x / rad_of_turn);

    // arc length is turning radius times angle
    return std::min(obstacle_free_path_len, arc_angle * rad_of_turn);
}

std::pair<double, double> Navigation::chooseCurvatureForNextTimestep(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map) {

    // TODO should we change the free path length to the one that will get us closest to the intermediate goal or keep
    //  it as just obstacle free?
    std::vector<double> reasonably_open_curvatures = getCurvaturesWithReasonablyOpenPaths(
            curvature_and_obstacle_limitations_map);

    double best_curvature;
    if (reasonably_open_curvatures.empty()) {

        // If there are no reasonably open curvatures, use the fallback weighing function to weigh between the free
        // path length and other considerations (proximity to goal, clearance)
        best_curvature = chooseCurvatureForNextTimestepNoOpenOptions(curvature_and_obstacle_limitations_map);
    } else {

        // Get the smallest curvature, since that is the most direct path to the goal.
        best_curvature = kMaxCurvature;
        for (const double &curvature : reasonably_open_curvatures) {
            if (abs(curvature) <= abs(best_curvature)) {
                if (abs(curvature) == abs(best_curvature)) {
                    // If they're the same curvature, but only left vs right, use the one with the higher clearance
                    if (curvature_and_obstacle_limitations_map.at(curvature).second >
                    curvature_and_obstacle_limitations_map.at(best_curvature).second) {
                        best_curvature = curvature;
                    }
                } else {
                    best_curvature = curvature;
                }
            }
        }
    }

    return std::make_pair(best_curvature,
            getFreePathLengthToClosestPointOfApproach(kIntermediateGoalX, best_curvature,
                    curvature_and_obstacle_limitations_map.at(best_curvature).first));
}

std::vector<double> Navigation::getCurvaturesWithReasonablyOpenPaths(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map) {
    std::vector<double> reasonably_open_curvatures;
    for (const auto &curvature_info : curvature_and_obstacle_limitations_map) {
        double curvature = curvature_info.first;
        double distance = curvature_info.second.first;
        double clearance = curvature_info.second.second;

        if ((distance >= open_free_path_len_threshold_) && (clearance >= open_clearance_threshold_)) {
            reasonably_open_curvatures.emplace_back(curvature);
        }
    }

    return reasonably_open_curvatures;
}

double Navigation::chooseCurvatureForNextTimestepNoOpenOptions(std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map) {

    std::pair<double, double> best_curvature_and_score = std::make_pair(kMaxCurvature, -1 * std::numeric_limits<double>::infinity());

    for (const auto &curvature_info : curvature_and_obstacle_limitations_map) {
        double curvature = curvature_info.first;
        double distance = curvature_info.second.first;
        double clearance = curvature_info.second.second;

        double score = scoreCurvature(curvature, distance, clearance);
        if (best_curvature_and_score.second < score) {
            best_curvature_and_score = std::make_pair(curvature, score);
        }
    }
    return best_curvature_and_score.first;
}

double Navigation::scoreCurvature(const double &curvature, const double &free_path_len, const double &clearance) {
    return free_path_len + (scoring_clearance_weight_ * clearance) + (scoring_curvature_weight_ * abs(curvature));
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
  visualization::ClearVisualizationMsg(local_viz_msg_);

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

  std::pair<double, double> curvature_and_dist_to_execute =
          chooseCurvatureForNextTimestep(free_path_len_and_clearance_by_curvature);
  executeTimeOptimalControl(curvature_and_dist_to_execute.second, curvature_and_dist_to_execute.first);

  for (const auto &curvature_info : free_path_len_and_clearance_by_curvature) {
      visualization::DrawPathOption(curvature_info.first, curvature_info.second.first,
                                    curvature_info.second.second, local_viz_msg_);
  }
  viz_pub_.publish(local_viz_msg_);
}


}  // namespace navigation
