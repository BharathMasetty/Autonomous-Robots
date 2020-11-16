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
using namespace std;

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
    nav_goal_angle_(0),
    map_file_(map_file) {
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

  // This initializes the vector with the correct number of commands with curvature 0 and velocity 0
  recent_executed_commands.resize(kNumActLatencySteps);

  map_.Load(map_file_);
  createNavGraph();
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;
    navigation_graph_.createUnalignedNode(loc, angle, 1);
}

void Navigation::UpdateLocation(const Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  cloud_ = cloud;
  scan_time_ = time; 
}

void Navigation::createNavGraph(){
  navigation_graph_.createNavigationGraph(map_);
  is_nav_graph_ready_ = true;
  ROS_INFO("Navigation graph created!");
  navigation_graph_.visualizeNavigationGraphPoints(0x34b4eb, global_viz_msg_);
  ROS_INFO("Graph Points visualized!");
  navigation_graph_.visualizeNavigationGraphEdges(0x34b4eb, global_viz_msg_);
  ROS_INFO("Graph Edges visualized!");
}

void Navigation::addCarDimensionsAndSafetyMarginAtPosToVisMessage(
        const std::pair<Eigen::Vector2f, double> &car_origin_loc, const uint32_t &car_color_loc,
        const uint32_t &safety_color, amrl_msgs::VisualizationMsg &viz_msg) {

    // Draw the car
    Vector2f back_left_corner(-kAxleToRearDist, (-0.5 * w));
    Vector2f front_left_corner(kAxleToFrontDist, (-0.5 * w));
    Vector2f front_right_corner(kAxleToFrontDist, (0.5 * w));
    Vector2f back_right_corner(-kAxleToRearDist, (0.5 * w));

    visualization::DrawLine(transformLocation(back_left_corner, car_origin_loc), transformLocation(front_left_corner, car_origin_loc), car_color_loc, viz_msg);
    visualization::DrawLine(transformLocation(front_left_corner, car_origin_loc), transformLocation(front_right_corner, car_origin_loc), car_color_loc, viz_msg);
    visualization::DrawLine(transformLocation(front_right_corner, car_origin_loc), transformLocation(back_right_corner, car_origin_loc), car_color_loc, viz_msg);
    visualization::DrawLine(transformLocation(back_right_corner, car_origin_loc), transformLocation(back_left_corner, car_origin_loc), car_color_loc, viz_msg);

    // Draw the safety margin
    Vector2f safety_back_left_corner(back_left_corner.x() - m, back_left_corner.y() - m);
    Vector2f safety_front_left_corner(front_left_corner.x() + m, front_left_corner.y() - m);
    Vector2f safety_front_right_corner(front_right_corner.x() + m, front_right_corner.y() + m);
    Vector2f safety_back_right_corner(back_right_corner.x() - m, back_right_corner.y() + m);

    visualization::DrawLine(transformLocation(safety_back_left_corner, car_origin_loc), transformLocation(safety_front_left_corner, car_origin_loc), safety_color, viz_msg);
    visualization::DrawLine(transformLocation(safety_front_left_corner, car_origin_loc), transformLocation(safety_front_right_corner, car_origin_loc), safety_color, viz_msg);
    visualization::DrawLine(transformLocation(safety_front_right_corner, car_origin_loc), transformLocation(safety_back_right_corner, car_origin_loc), safety_color, viz_msg);
    visualization::DrawLine(transformLocation(safety_back_right_corner, car_origin_loc), transformLocation(safety_back_left_corner, car_origin_loc), safety_color, viz_msg);
}

void Navigation::addCarDimensionsAndSafetyMarginToVisMessage(amrl_msgs::VisualizationMsg &viz_msg) {
    addCarDimensionsAndSafetyMarginAtPosToVisMessage(std::make_pair(Vector2f(0.0, 0.0), 0.0), kCarBoundariesColor, kCarSafetyMarginColor, viz_msg);
}

Eigen::Vector2f Navigation::transformLocation(const Eigen::Vector2f &loc_to_transform,
        const std::pair<Eigen::Vector2f, double> &transform_info) {
    double angle = transform_info.second;
    double x = transform_info.first.x() + cos(angle) * loc_to_transform.x() - sin(angle) * loc_to_transform.y();
    double y = transform_info.first.y() + sin(angle) * loc_to_transform.x() + cos(angle) * loc_to_transform.y();

    return Vector2f(x, y);
}

void Navigation::drawCarPosAfterCurvesExecuted(
        const std::unordered_map<double, std::pair<double, double>> &free_path_len_and_clearance_by_curvature) {
    for (const auto &curvature_info : free_path_len_and_clearance_by_curvature) {
        std::pair<Vector2f, double> car_pos_after_path =
                getLocationAfterCurvatureExecution(curvature_info.first, curvature_info.second.first);

        uint32_t car_color = kPredictedCarBoundariesColor;
        uint32_t safety_color = kPredictedCarSafetyMarginColor;
        if (isPathReasonablyOpen(curvature_info.second.first, curvature_info.second.second)) {
            car_color = kPredictedOpenPathCarBoundariesColor;
            safety_color = kPredictedOpenPathSafetyMarginColor;
        }
        addCarDimensionsAndSafetyMarginAtPosToVisMessage(car_pos_after_path, car_color,
                safety_color, local_viz_msg_);
    }
}

std::pair<Eigen::Vector2f, double> Navigation::getLocationAfterCurvatureExecution(const double &curvature, const double &path_len) {
    if (curvature == 0.0) {
        return std::make_pair(Vector2f(path_len, 0), 0);
    }

    double path_arc_angle = abs(path_len * curvature); // equivalent to path_len / radius
    double radius = abs(1 / curvature);
    double x = sin(path_arc_angle) * radius;
    double y = radius - (cos(path_arc_angle) * radius);
    if (curvature < 0) {
        y = -y;
        path_arc_angle = -path_arc_angle;
    }
    return std::make_pair(Vector2f(x, y), path_arc_angle);
}

double Navigation::computeDecelDistance(const double &velocity_to_decelerate_from) {
    return std::pow(velocity_to_decelerate_from, 2) / (2 * std::abs(kMaxDecel));
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
        ROS_INFO("No reasonably open paths");

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

bool Navigation::isPathReasonablyOpen(const double &free_path_len, const double &clearance) {
    if ((free_path_len >= open_free_path_len_threshold_) && (clearance >= open_clearance_threshold_)) {
        return true;
    }
    return false;
}

std::vector<double> Navigation::getCurvaturesWithReasonablyOpenPaths(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map) {
    std::vector<double> reasonably_open_curvatures;
    for (const auto &curvature_info : curvature_and_obstacle_limitations_map) {
        double curvature = curvature_info.first;
        double distance = curvature_info.second.first;
        double clearance = curvature_info.second.second;

        if (isPathReasonablyOpen(distance, clearance)) {
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

    std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature;
    for (const double &curvature : curvatures_to_evaluate) {
        free_path_len_and_clearance_by_curvature[curvature] = getFreePathLengthAndClearance(curvature);
    }
    return free_path_len_and_clearance_by_curvature;
}

std::pair<double, double> Navigation::getFreePathLengthAndClearance(const double &curvature) {

    // TODO: Bharath: Keep thinking of a better way to do this 
    float free_path_len = std::numeric_limits<float>::infinity();
    float clearance = std::numeric_limits<float>::infinity();
    float r;
    float r_min;
    float r_max;
    float r_fc;
    float r_p;
    float pp_hit;
    float theta;

    // Center of rotation
    Eigen::Vector2f IC;
    // To search in part of point cloud
    const int startIndex = 60; // -90 Degrees wrt base   
    const int endIndex = 1020;   // +90 Degrees wrt base
    // TODO: Probably make this a ROS Param Later as needed.
    const double kClearanceOffset = 0.1; 
    std::vector<double> notHitting_r;
    // aplha is angle made by r and r_p at IC
    // We are keeping track of this extra angle for notHitting points and 
    // free_path_len point to reduce search space for clearance.
    std::vector<double> notHitting_alpha;
    // Initializing freePathalpha at pi radians
    double freePathalpha = 3.14;
    float hittingSign = abs(curvature)/curvature; 
    
    if (abs(curvature) > 0) {
        IC.x() = 0.0;
        IC.y() = 1/curvature;
        r = abs(1 / curvature);

        // Set the free path length to the the circumference of the circle around the center of turning (max)
        free_path_len = 2.0 * M_PI * r;
        r_min = r - kLengthFromBaseToSafetySide;
	float r_min_square = std::pow(r_min, 2);
        r_max = std::pow(std::pow(kLengthFromAxleToSafetyFront, 2) + std::pow(r+ kLengthFromBaseToSafetySide, 2) , 0.5);
	r_fc = std::pow(std::pow(kLengthFromAxleToSafetyFront, 2) + std::pow(r - kLengthFromBaseToSafetySide, 2) , 0.5);
    
        // Converting cloud to polar
        for(int i=startIndex; i<=endIndex; i++) {
            float r_p_square = std::pow(cloud_[i].x(), 2) + std::pow(cloud_[i].y() - IC.y(), 2);
            r_p = std::pow(r_p_square, 0.5);
            double op = std::pow(std::pow(cloud_[i].x(), 2) + std::pow(cloud_[i].y(), 2), 0.5);

            // First check for free_path_length
            if (r_p >= r_min && r_p <= r_max) {

                // Side Hit
                if (r_p < r_fc){
                    float x = std::pow(r_p_square - r_min_square, 0.5);
		    pp_hit = std::pow(cloud_[i].x() - x ,2) + std::pow(cloud_[i].y() - kLengthFromBaseToSafetySide*hittingSign, 2);
                }

                // Front Hit
                if ( r_p >= r_fc) {
                    float y = hittingSign*(r - std::pow(r_p_square - std::pow(kLengthFromAxleToSafetyFront, 2) ,0.5)) ;
                    pp_hit = std::pow(cloud_[i].x()- kLengthFromAxleToSafetyFront, 2) + std::pow(cloud_[i].y()-y, 2);
                }

                theta =  acos(1 - pp_hit/(2*r_p_square));
                float len = r*theta;

                if (len <= free_path_len){
                    free_path_len = len;
                    freePathalpha = acos((r*r + r_p_square - op*op)/(2*r*r_p));
                }
            } else {
                // Stacking up Non-Htting points
                notHitting_r.push_back(r_p);
                notHitting_alpha.push_back(acos((r*r + r_p_square - op*op)/(2*r*r_p)));
            }
        }
 	
        // Next Check  Clearance
        for (unsigned int i=0; i<=notHitting_r.size(); i++) {
            double possible_new_clearance = std::min(abs(notHitting_r[i] - r_min), abs(notHitting_r[i] - r_max));
            if (free_path_len > kClearanceOffset) {
                if (r*(freePathalpha - notHitting_alpha[i]) >= kClearanceOffset) {
                    clearance = std::min((double)clearance, possible_new_clearance);
                }
            } else if (notHitting_alpha[i] < freePathalpha) {
                clearance = std::min((double)clearance, possible_new_clearance);
            }
        }
    }

    // When moving along a straight line
    if (curvature == 0.0) {
        // default
	std::vector<Eigen::Vector2f> notHittingPoints;
        for (int i=startIndex; i<=endIndex; i++) {
            if (abs(cloud_[i].y()) <= kLengthFromBaseToSafetySide) {
                free_path_len = std::min(free_path_len, cloud_[i].x() - kLengthFromAxleToSafetyFront);
            }else{
	    	notHittingPoints.push_back(cloud_[i]);
	    }
        }
        for (unsigned int i=0; i<=notHittingPoints.size(); i++) {
            double new_clearance = abs(notHittingPoints[i].y()) - kLengthFromBaseToSafetySide;

            if (free_path_len > kClearanceOffset){
                if((notHittingPoints[i].x()- kLengthFromAxleToSafetyFront) <= free_path_len - kClearanceOffset) {
                    clearance = std::min((double) clearance, new_clearance);
                }
            } else if((notHittingPoints[i].x()-kLengthFromAxleToSafetyFront) <= free_path_len){
                clearance = std::min((double) clearance, new_clearance);
            }
        }
    }
    
    return std::make_pair(free_path_len, clearance);
}

void Navigation::executeTimeOptimalControl(const double &distance, const double &curvature) {

    // Get the velocity for each of the timesteps that we need to account for in the latency compensation
    double compensation_velocity_sum = 0;
    for (int i = 0; i < kNumActLatencySteps; i++) {
        compensation_velocity_sum += recent_executed_commands[i].velocity;
    }

    double compensation_distance = kLoopExecutionDelay * compensation_velocity_sum;
    double distance_remaining = std::max(0.0, distance - compensation_distance);

    double current_velocity = recent_executed_commands[0].velocity;
    if (distance_remaining > kStopDist) {

        // Implementing 1-D TOC on an arbitrary arc, the parameters to be considered are new_distances and curvature
        if (computeDecelDistance(current_velocity) >= distance_remaining) {
            // Decelerate
            current_velocity = std::max(0.0, kLoopExecutionDelay * kMaxDecel + current_velocity);
        } else {
            // Latency compensation
            if (current_velocity < kMaxVel) {
                // Accelerate
                current_velocity = std::min(kMaxVel, kMaxAccel * kLoopExecutionDelay + current_velocity);
            }
            // If not accelerating, we'll keep the same velocity as before
        }
    } else {
        ROS_INFO_STREAM("Within target distance");
        current_velocity = 0.0;
    }

    AckermannCurvatureDriveMsg drive_msg;
    drive_msg.velocity = current_velocity;
    drive_msg.curvature = curvature;
    drive_pub_.publish(drive_msg);

    for (int i = kNumActLatencySteps - 1; i > 0; i--){
        recent_executed_commands[i] = recent_executed_commands[i - 1];
    }
    recent_executed_commands[0] = drive_msg;
}

std::pair<Eigen::Vector2f, float> Navigation::getCarrot() {
    // This function assumes that the first node in the global plan is reachable and that there is at least one node
    // in the global plan

    // Since this connects nodes using straight lines instead of lattice and doesn't expand the map, this also assumes
    // that the local planner will be able to steer away from walls enough to handle this approximation.
    nav_graph::NavGraphNode last_reachable_node = global_plan_to_execute_[0];
    for (size_t i = 1; i < global_plan_to_execute_.size(); i++) {
        if (!map_.Intersects(global_plan_to_execute_[i].getNodePos(), robot_loc_)) {
            last_reachable_node = global_plan_to_execute_[i];
        } else {
            break;
        }
    }
    return std::make_pair(last_reachable_node.getNodePos(), last_reachable_node.getNodeOrientation());
}


bool Navigation::planStillValid() {
    // TODO should also prune nodes that we've gone past (so we only have nodes left to traverse in global plan)
    return true; // Amanda TODO
}

void Navigation::runObstacleAvoidance(const std::pair<Eigen::Vector2f, float> &carrot) {

    std::vector<double> curvatures_to_evaluate = getCurvaturesToEvaluate();

    std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature =
            getFreePathLengthsAndClearances(curvatures_to_evaluate);

    std::pair<double, double> curvature_and_dist_to_execute =
            chooseCurvatureForNextTimestep(free_path_len_and_clearance_by_curvature);
    executeTimeOptimalControl(curvature_and_dist_to_execute.second, curvature_and_dist_to_execute.first);

    for (const auto &curvature_info : free_path_len_and_clearance_by_curvature) {
        if (curvature_info.first != curvature_and_dist_to_execute.first) {
            visualization::DrawPathOption(curvature_info.first, curvature_info.second.first,
                                          curvature_info.second.second, local_viz_msg_);
        }
    }

    drawCarPosAfterCurvesExecuted(free_path_len_and_clearance_by_curvature);
    // Add the best curvature last so it is highlighted in the visualization
    double best_curvature = curvature_and_dist_to_execute.first;
    std::pair<double, double> best_curvature_info = free_path_len_and_clearance_by_curvature[best_curvature];
    visualization::DrawPathOption(best_curvature, best_curvature_info.first, best_curvature_info.second+kLengthFromBaseToSafetySide, local_viz_msg_);
}

void Navigation::Run() {
  visualization::ClearVisualizationMsg(local_viz_msg_);
  addCarDimensionsAndSafetyMarginToVisMessage(local_viz_msg_);

  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.

  if (drive_pub_.getNumSubscribers() == 0) {
    ROS_ERROR("Still no subscribers to Drive message. Not yet sending velocities.");
    return;
  }

  // TODO check if reached goal (Kunal)

  //if (!planStillValid()) {
      // TODO replan
  //}

  //std::pair<Eigen::Vector2f, float> carrot = getCarrot();
  //runObstacleAvoidance(carrot);
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

void Navigation::ReachedGoal(){
    if ((nav_goal_loc_ - robot_loc_).norm() < kGoalTolerance){
        nav_complete_ = true;
    }
}
}  // namespace navigation
