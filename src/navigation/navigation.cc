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
    map_file_(map_file),
    new_goal_(false) {
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
  computeInflatedMap(map_, inflated_map_);

  createNavGraph();
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::computeInflatedMap(const vector_map::VectorMap &original_map, vector_map::VectorMap &new_map) {
    std::vector<geometry::line2f> new_lines;
    for (const geometry::line2f &line : original_map.lines) {
        Vector2f unit_vec = (line.p1 - line.p0) / line.Length();
        Vector2f perp_vec = geometry::Perp(unit_vec);
        new_lines.emplace_back(geometry::line2f(line.p0 - (kMapInflationAmount * perp_vec), line.p1 - (kMapInflationAmount * perp_vec)));
        new_lines.emplace_back(geometry::line2f(line.p0 + (kMapInflationAmount * perp_vec), line.p1 + (kMapInflationAmount * perp_vec)));
        new_lines.emplace_back(geometry::line2f(line.p0 - (kMapInflationAmount * unit_vec), line.p1 + (kMapInflationAmount * unit_vec)));
    }
    new_map = vector_map::VectorMap(new_lines);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    ROS_INFO_STREAM("Setting new goal " << loc.x() << ", " << loc.y());
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;
    new_goal_ = true;
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
  permanent_navigation_graph_.createNavigationGraph(map_, inflated_map_);
  is_nav_graph_ready_ = true;
  ROS_INFO("Navigation graph created!");
  //  permanent_navigation_graph_.visualizeNavigationGraphPoints(global_viz_msg_);
  ROS_INFO("Graph Points visualized!");
  //  permanent_navigation_graph_.visualizeNavigationGraphEdges(global_viz_msg_);
  ROS_INFO("Graph Edges visualized!");
  viz_pub_.publish(global_viz_msg_);
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
        double abs_curv = (((double) i ) / half_num_to_eval) * nav_graph::kMaxCurvature;
        eval_curves[pos_curve_index] = abs_curv;
        eval_curves[neg_curve_index] = -1 * abs_curv;
    }
    return eval_curves;
}

std::vector<double> Navigation::getCurvaturesToEvaluate() {
    return curvatures_to_evaluate_;
}

std::unordered_map<double, double> Navigation::getPathLengthForClosestPointOfApproach(
        const Vector2f &carrot, const std::vector<double> &curvatures) {
    std::unordered_map<double, double> path_len_for_closest_point_of_approach;
    for (const double &curvature : curvatures) {
        if (curvature == 0.0) {
            path_len_for_closest_point_of_approach[curvature] = carrot.x();
        } else {
            double rad_of_turn = 1.0 / curvature;
            double x_target = carrot.x();
            double y_target = carrot.y();
            if (rad_of_turn < 0) {
                y_target = -1 * y_target;
            }
            rad_of_turn = abs(rad_of_turn);

            double numerator = pow(rad_of_turn, 2) + pow(x_target, 2) + pow(y_target - rad_of_turn, 2) -
                    carrot.squaredNorm();
            double denom = 2 * rad_of_turn * sqrt((pow(x_target, 2) + pow(y_target - rad_of_turn, 2)));
            double angle = acos(numerator / denom);

            // Make sure angle is in right quadrant
            if (x_target < 0) {
                angle = M_2_PI - angle;
            }

            // Arc length is radius times angle
            path_len_for_closest_point_of_approach[curvature] = rad_of_turn * angle;
        }
    }
    return path_len_for_closest_point_of_approach;
}

double Navigation::getOptimalCurvatureForCarrot(const Eigen::Vector2f &carrot_location) {
    double curvature = (2 * carrot_location.y()) / carrot_location.squaredNorm();

    // Make sure curvature is within bounds
    curvature = std::max(-1 * nav_graph::kMaxCurvature, std::min(nav_graph::kMaxCurvature, curvature));
    return curvature;
}

std::pair<double, double> Navigation::chooseCurvatureForNextTimestep(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map,
        const std::pair<Eigen::Vector2f, float> &carrot) {

    // Get the curvatures with reasonably open paths
    // These are already limited by the closest point of approach to the goal, so we don't need to consider if we'll
    // overshoot the goal.
    std::vector<double> reasonably_open_curvatures = getCurvaturesWithReasonablyOpenPaths(curvature_and_obstacle_limitations_map);

    double best_curvature;
    if (reasonably_open_curvatures.empty()) {
//        ROS_INFO("Close to carrot or no reasonably open paths");

        // If there are no reasonably open curvatures, use the fallback weighing function to weigh between the free
        // path length and other considerations (proximity to goal, clearance)
        best_curvature = chooseCurvatureForNextTimestepNoOpenOptions(curvature_and_obstacle_limitations_map, carrot);
    } else {

        // Find the optimal curvature and then pick the one that is resonably open that is closest to that optimal
        // curvature.
        best_curvature = nav_graph::kMaxCurvature;
        double diff_from_best_curvature = std::numeric_limits<double>::infinity();
        double optimal_curvature_for_carrot = getOptimalCurvatureForCarrot(carrot.first);

        for (const double &curvature : reasonably_open_curvatures) {
            float diff_from_curvature = abs(curvature - optimal_curvature_for_carrot);
            if (diff_from_curvature <= diff_from_best_curvature) {
                if (diff_from_curvature == diff_from_best_curvature) {
                    // If they're the same curvature, but only left vs right, use the one with the higher clearance
                    if (curvature_and_obstacle_limitations_map.at(curvature).second >
                        curvature_and_obstacle_limitations_map.at(best_curvature).second) {
                        best_curvature = curvature;
                        diff_from_best_curvature = diff_from_curvature;
                    }
                } else {
                    best_curvature = curvature;
                    diff_from_best_curvature = diff_from_curvature;
                }
            }
        }
    }

    // Don't need to shrink free path length to closest point of approach because max to closest point of approach was
    // already done before computing the free path length.
    return std::make_pair(best_curvature, curvature_and_obstacle_limitations_map.at(best_curvature).first);
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

double Navigation::chooseCurvatureForNextTimestepNoOpenOptions(
        std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map,
        const std::pair<Eigen::Vector2f, float> &carrot) {

    std::pair<double, double> best_curvature_and_score = std::make_pair(
            nav_graph::kMaxCurvature, -1 * std::numeric_limits<double>::infinity());

    double best_curvature = getOptimalCurvatureForCarrot(carrot.first);

    for (const auto &curvature_info : curvature_and_obstacle_limitations_map) {
        double curvature = curvature_info.first;
        double distance = curvature_info.second.first;
        double clearance = curvature_info.second.second;

        double score = scoreCurvature(curvature, distance, clearance, best_curvature);
        if (best_curvature_and_score.second < score) {
            best_curvature_and_score = std::make_pair(curvature, score);
        }
    }
    return best_curvature_and_score.first;
}

double Navigation::scoreCurvature(const double &curvature, const double &free_path_len, const double &clearance, const double &optimal_curvature) {
    double very_small_clearance_penalty = 0.0;
    if (clearance < kSmallClearanceThreshold) {
        very_small_clearance_penalty = kSmallClearancePenalty;
    }
    return very_small_clearance_penalty + scoring_free_path_weight_*free_path_len + (scoring_clearance_weight_ * clearance) + (scoring_curvature_weight_ * abs(curvature - optimal_curvature));
}

std::unordered_map<double, std::pair<double, double>> Navigation::getFreePathLengthsAndClearances(
        const std::unordered_map<double, double> &curvatures_and_free_path_len_for_closest_point_of_approach,  const std::vector<Vector2f>& cloud) {

    std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature;
    for (const auto &curvature_and_closest_approach_arc_len : curvatures_and_free_path_len_for_closest_point_of_approach) {
        double curvature = curvature_and_closest_approach_arc_len.first;
        double free_path_len_for_closest_point_of_approach = curvature_and_closest_approach_arc_len.second;
        free_path_len_and_clearance_by_curvature[curvature] = getFreePathLengthAndClearance(curvature, free_path_len_for_closest_point_of_approach, cloud);
    }
    return free_path_len_and_clearance_by_curvature;
}

std::pair<double, double> Navigation::getFreePathLengthAndClearance(const double &curvature, 
								    const double &free_path_len_for_closest_point_of_approach,
								    const std::vector<Vector2f>& cloud) {

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
        free_path_len = free_path_len_for_closest_point_of_approach;
        r_min = r - kLengthFromBaseToSafetySide;
        float r_min_square = std::pow(r_min, 2);
        r_max = std::pow(std::pow(kLengthFromAxleToSafetyFront, 2) + std::pow(r+ kLengthFromBaseToSafetySide, 2) , 0.5);
        r_fc = std::pow(std::pow(kLengthFromAxleToSafetyFront, 2) + std::pow(r - kLengthFromBaseToSafetySide, 2) , 0.5);
    
        // Converting cloud to polar
        for(int i=startIndex; i<=endIndex; i++) {
            float r_p_square = std::pow(cloud[i].x(), 2) + std::pow(cloud[i].y() - IC.y(), 2);
            r_p = std::pow(r_p_square, 0.5);
            double op = std::pow(std::pow(cloud[i].x(), 2) + std::pow(cloud[i].y(), 2), 0.5);

            // First check for free_path_length
            if (r_p >= r_min && r_p <= r_max) {

                // Side Hit
                if (r_p < r_fc){
                    float x = std::pow(r_p_square - r_min_square, 0.5);
                    pp_hit = std::pow(cloud[i].x() - x ,2) + std::pow(cloud[i].y() - kLengthFromBaseToSafetySide*hittingSign, 2);
                }

                // Front Hit
                if ( r_p >= r_fc) {
                    float y = hittingSign*(r - std::pow(r_p_square - std::pow(kLengthFromAxleToSafetyFront, 2) ,0.5)) ;
                    pp_hit = std::pow(cloud[i].x()- kLengthFromAxleToSafetyFront, 2) + std::pow(cloud[i].y()-y, 2);
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
        free_path_len = free_path_len_for_closest_point_of_approach;
        // default
        std::vector<Eigen::Vector2f> notHittingPoints;
        for (int i=startIndex; i<=endIndex; i++) {
            if (abs(cloud[i].y()) <= kLengthFromBaseToSafetySide) {
                free_path_len = std::min(free_path_len, cloud[i].x() - kLengthFromAxleToSafetyFront);
            } else{
                notHittingPoints.push_back(cloud[i]);
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
    /*
    double compensation_velocity_sum = 0;
    for (int i = 0; i < kNumActLatencySteps; i++) {
        compensation_velocity_sum += recent_executed_commands[i].velocity;
    }

    double compensation_distance = kLoopExecutionDelay * compensation_velocity_sum;
    double distance_remaining = std::max(0.0, distance - compensation_distance);
    */
    double distance_remaining = distance;
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
    // This function assumes that the first node in the global_plan_to_execute_ is the one that we are coming from and
    // the second node is reachable and is the one that we are approaching.

    // Since this connects nodes using straight lines instead of lattice and doesn't expand the map, this also assumes
    // that the local planner will be able to steer away from walls enough to handle this approximation.
    std::pair<Vector2f, float> prev_reachable_node = std::make_pair(robot_loc_, robot_angle_);
    std::pair<Vector2f, float> last_reachable_node = std::make_pair(global_plan_to_execute_[1].getNodePos(), global_plan_to_execute_[1].getNodeOrientation());
    bool last_node_goal = false;
    bool not_start = false;
    for (size_t i = 2; i < global_plan_to_execute_.size(); i++) {
        if ((!map_.Intersects(global_plan_to_execute_[i].getNodePos(), robot_loc_)) && ((global_plan_to_execute_[i].getNodePos() - robot_loc_).norm() < kMaxCarrotDistance)) {

            Vector2f node_pos_rel_robot = nav_graph::inverseTransformPoint(global_plan_to_execute_[i].getNodePos(), 0, robot_loc_, robot_angle_).first;
            // TODO Consider checking if curvature is executable (greater than or equal to 1). That might be better than checking for a positive x value.

            if (node_pos_rel_robot.x() >= kMinCarrotXRelCar) {
                not_start = true;
                prev_reachable_node = last_reachable_node;
                last_reachable_node = std::make_pair(global_plan_to_execute_[i].getNodePos(), global_plan_to_execute_[i].getNodeOrientation());
                last_node_goal = !global_plan_to_execute_[i].isGridAligned();
            }
        } else {
            break;
        }
    }

    if ((!last_node_goal) && (not_start)){
        if (kAngularCarrotMaxChange < math_util::AngleDist(last_reachable_node.second, prev_reachable_node.second)) {
            float angle_diff = AngleDiff(last_reachable_node.second, prev_reachable_node.second);
            double radius =
                     (last_reachable_node.first - prev_reachable_node.first).norm() / sqrt(2 * (1 - cos(angle_diff)));
            float target_angle_change = kAngularCarrotMaxChange;
            if (angle_diff < 0) {
                target_angle_change = -kAngularCarrotMaxChange;
                radius = -1 * radius;
            } else {
                target_angle_change = -kAngularCarrotMaxChange;
            }

            std::pair<Vector2f, float> carrot_node_in_prev_node_frame = std::make_pair(Vector2f(
                    abs(radius) * sin(abs(target_angle_change)), radius - (radius * cos(abs(target_angle_change)))),
                                                                                       target_angle_change);
            std::pair<Vector2f, float> carrot_in_map = nav_graph::transformPoint(carrot_node_in_prev_node_frame.first,
                                                                                 carrot_node_in_prev_node_frame.second,
                                                                                 prev_reachable_node.first, prev_reachable_node.second);
            return nav_graph::inverseTransformPoint(carrot_in_map.first, carrot_in_map.second,
                                                    robot_loc_, robot_angle_);
        }
    }
    // Convert the carrot into the base link frame
    return nav_graph::inverseTransformPoint(last_reachable_node.first, last_reachable_node.second,
                                 robot_loc_, robot_angle_);
}

bool Navigation::isCarInBetweenNodes(const nav_graph::NavGraphNode &node_1, const nav_graph::NavGraphNode &node_2) {

    // Check if the car is in the rectangle centered around the line connecting node 1 and node 2
    geometry::line2f line_between_nodes(node_1.getNodePos(), node_2.getNodePos());
    double line_angle = atan2(line_between_nodes.Dir().y(), line_between_nodes.Dir().x());
    Vector2f car_in_line_frame = nav_graph::inverseTransformPoint(robot_loc_, robot_angle_, node_1.getNodePos(), line_angle).first;

    if ((car_in_line_frame.x() < 0) || (car_in_line_frame.x() >= line_between_nodes.Length())) {
        return false;
    }

    if (abs(car_in_line_frame.y()) > kLateralDeviationFromPlanAllowance) {
        return false;
    }

    // Car is positioned between the nodes, want to also check the angle
    float source_node_angle = AngleMod(node_1.getNodeOrientation());
    float dest_node_angle = AngleMod(node_2.getNodeOrientation());
    float angle_diff = AngleMod(dest_node_angle - source_node_angle);

    float target_robot_angle = AngleMod(source_node_angle + (angle_diff * (car_in_line_frame.x() / line_between_nodes.Length())));
    float angle_dist = AngleDist(target_robot_angle, AngleMod(robot_angle_));

    if ((angle_dist > kAngularDeviationFromPlanAllowance) && node_2.isGridAligned()) {
        return false;
    }

    return true;
}

void Navigation::drawCarrot(const Eigen::Vector2f &carrot) {
    visualization::DrawCross(carrot, kCarrotCrossSize, kCarrotColor, local_viz_msg_);
}

bool Navigation::planStillValid() {

    if (global_plan_to_execute_.empty()) {
        ROS_ERROR_STREAM("Could not find path to goal. Need to set new goal (and investigate why we couldn't find path");
        return false;
    }

    bool is_between_nodes = false;
    std::size_t immediately_preceding_index;
    for (std::size_t i = 0; i < global_plan_to_execute_.size() -1; i++) {
        if (isCarInBetweenNodes(global_plan_to_execute_[i], global_plan_to_execute_[i+1])) {
            immediately_preceding_index = i;
            is_between_nodes = true;
        }
    }
    if (!is_between_nodes) {
        return false;
    }
    if (immediately_preceding_index != 0) {
        // Remove nodes before immediately_preceding_index
        std::vector<decltype(global_plan_to_execute_)::value_type>(
                global_plan_to_execute_.begin() + immediately_preceding_index,
                global_plan_to_execute_.end()).swap(global_plan_to_execute_);
    }

    // Check that the first node in the trajectory (not including the one that we've just passed) is reachable from the
    // current position
    // TODO Validate this. It's possible that if the edge is a curve, that won't intersect even if the straight line
    //  connection (used here) does, so perhaps it could get stuck.
    if (global_plan_to_execute_.size() > 1) {
        if (map_.Intersects(robot_loc_, global_plan_to_execute_[1].getNodePos())) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

void Navigation::runObstacleAvoidance(const std::pair<Eigen::Vector2f, float> &carrot) {
 	
    std::vector<Vector2f> compensatedCloud = transformCloudForHighSpeeds(); 
    std::vector<double> curvatures_to_evaluate = getCurvaturesToEvaluate();
    std::unordered_map<double, double> free_path_len_for_closest_point_of_approach = getPathLengthForClosestPointOfApproach(
            carrot.first, curvatures_to_evaluate);

    std::unordered_map<double, std::pair<double, double>> free_path_len_and_clearance_by_curvature =
            getFreePathLengthsAndClearances(free_path_len_for_closest_point_of_approach, compensatedCloud);

    std::pair<double, double> curvature_and_dist_to_execute =
            chooseCurvatureForNextTimestep(free_path_len_and_clearance_by_curvature, carrot);
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

void Navigation::displayGlobalPath() {
    for (size_t i = 1; i < global_plan_to_execute_.size(); i++) {
        visualization::DrawCross(global_plan_to_execute_[i].getNodePos(), kGlobalPlanCrossSize,
                                 kGlobalPlanColor, global_viz_msg_);
        // TODO we should probably draw the arcs instead of straight line connections.
        visualization::DrawLine(global_plan_to_execute_[i - 1].getNodePos(), global_plan_to_execute_[i].getNodePos(),
                                kGlobalPlanColor, global_viz_msg_);
    }
}

void Navigation::Run() {
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  addCarDimensionsAndSafetyMarginToVisMessage(local_viz_msg_);
//
//  for (const geometry::line2f &inflated_lines : inflated_map_.lines) {
//      visualization::DrawLine(inflated_lines.p0, inflated_lines.p1, 0x7b32a8, global_viz_msg_);
//  }

  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
  if (drive_pub_.getNumSubscribers() == 0) {
    ROS_ERROR("Still no subscribers to Drive message. Not yet sending velocities.");
    return;
  }
  if (cloud_.empty()) {
      ROS_ERROR("Still no point cloud. Not sending velocities.");
      return;
  }

  ReachedGoal();
  if (nav_complete_) {
      ROS_INFO_STREAM("No further to go. Will wait for new goal.");
      global_plan_to_execute_.clear();
      return;
  }

  if (global_plan_to_execute_.empty() || new_goal_) {
      new_goal_ = false;
      temp_node_nav_graph_ = permanent_navigation_graph_.createCopy();
      ROS_INFO_STREAM("Adding start and goal nodes");
      std::pair<nav_graph::NavGraphNode, nav_graph::NavGraphNode> start_goal_nodes = temp_node_nav_graph_.addStartAndGoal(
              std::make_pair(robot_loc_, robot_angle_), nav_goal_loc_);
      goal_node_ = start_goal_nodes.second;
      start_node_ = start_goal_nodes.first;
      ROS_INFO_STREAM("Planning path ");
      global_plan_to_execute_ = nav_graph::GetPathToGoal(goal_node_, start_goal_nodes.first, temp_node_nav_graph_);
      ROS_INFO_STREAM("Done planning path");
      // Plan
  }
//  ROS_INFO_STREAM("Has plan of length " << global_plan_to_execute_.size());
//  temp_node_nav_graph_.visualizeConnectionsFromNode(start_node_, global_viz_msg_);
//  temp_node_nav_graph_.visualizeGoalEdges(goal_node_.getNodePos(), global_viz_msg_);

//  temp_node_nav_graph_.visualizeNavigationGraphPoints( global_viz_msg_);
  displayGlobalPath();
  if (!planStillValid()) {
      ROS_INFO_STREAM("Replanning!");
      nav_graph::NavGraphNode updated_start = temp_node_nav_graph_.updateStart(std::make_pair(robot_loc_, robot_angle_));
      start_node_ = updated_start;
      global_plan_to_execute_ = nav_graph::GetPathToGoal(goal_node_, updated_start, temp_node_nav_graph_);
  }

  if (global_plan_to_execute_.empty()) {
      ROS_ERROR_STREAM("Could not find path to goal. Need to set new goal (and investigate why we couldn't find path");
  } else {
      std::pair<Eigen::Vector2f, float> carrot = getCarrot();
      drawCarrot(carrot.first);
      runObstacleAvoidance(carrot);
  }
  viz_pub_.publish(global_viz_msg_);
  viz_pub_.publish(local_viz_msg_);
}

void Navigation::ReachedGoal(){
    if ((nav_goal_loc_ - robot_loc_).norm() < kGoalTolerance) {
        nav_complete_ = true;
    }
}

std::vector<Vector2f> Navigation::transformCloudForHighSpeeds(){

    Vector2f curr_loc = robot_loc_;
    float curr_angle = robot_angle_;
    std::vector<Vector2f> cloud;
    cloud.resize(cloud_.size());
    for (uint32_t j=0; j< cloud.size(); j++){
    	cloud[j]  = cloud_[j];
     }
    
    for(uint32_t i=0; i<recent_executed_commands.size(); i++){
        AckermannCurvatureDriveMsg past_command = recent_executed_commands[i];
        double curvature = past_command.curvature;
        float velocity = past_command.velocity;
        if (past_command.curvature != 0){
                 float radius = std::abs(1/curvature);
                 Vector2f center(0.0, 1/curvature);
                 float turnSign = curvature/abs(curvature);
                 float angleAtCenter = -turnSign*M_PI_2;
                 float angleChange = velocity*kLoopExecutionDelay*curvature;
                 float newAngle = angleAtCenter+angleChange;
                 Vector2f compensatedLocInBaseFrame(center.x()+radius*cos(newAngle), center.y()+radius*sin(newAngle));
                 curr_loc = curr_loc + compensatedLocInBaseFrame;
                 curr_angle = curr_angle+angleChange;
                 Eigen::Rotation2Df rotate(-1*angleChange);
                 for (uint32_t j=0; j< cloud.size(); j++){
			cloud[j]  = rotate*cloud[j];
                        cloud[j] -= compensatedLocInBaseFrame;
                }
        }
        else{
                 Vector2f compensatedLocInBaseFrame(velocity*kLoopExecutionDelay, 0);
                 for (uint32_t j=0; j < cloud_.size(); j++){
			 cloud[j] -= compensatedLocInBaseFrame;
                 }
                 curr_loc = curr_loc + compensatedLocInBaseFrame;
        }
    }
    
    for (uint32_t j=0; j<cloud.size(); j++){
	visualization::DrawPoint(cloud[j], kPredictedCloudPointsColor, local_viz_msg_);
    }

    return cloud;
}


}  // namespace navigation
