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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <amrl_msgs/AckermannCurvatureDriveMsg.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <vector>
#include <unordered_map>
#include <navigation/nav_graph.h>
#include <vector_map/vector_map.h>

#include "eigen3/Eigen/Dense"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:

  /**
   * Max acceleration of the car.
   */
  const double kMaxAccel = 4.0;

  /**
   * Max deceleration of the car.
   */
  const double kMaxDecel = -4.0;

  /**
   * Max velocity of the car.
   */
  const double kMaxVel = 1.0;

  /**
   * If we have this distance remaining, say that we've met the goal.
   */
  const double kStopDist = 0.03;

  /**
   * Distance that we must get within to say we've reached the goal.
   */
  const double kGoalTolerance = 0.1;

  /**
   * Amount of time between each loop execution.
   */
  const double kLoopExecutionDelay = (1.0 / 20.0);

  /**
   * Actuation latency (time between when command is issued to when motors execute the command.
   */
  const double kActuationLatency = 0.15;

  /**
   * Number of curvatures to evaluate. These will be evenly spaced between c_min and c_max
   * (see kMaxCurvature).
   *
   * This should be a positive odd number >= 3, so we can evaluate straight as an option.
   */
  const int kNumCurvaturesToEval = 41;

  /**
   * Default value for the clearance that a path must have to be considered "reasonably open".
   */
  const double kDefaultDesiredClearance = 0.2;

  /**
   * Amount to inflate map by. Lines will be 2x this much longer and lines will be added to each side of each line this
   * far away.
   */
  const double kMapInflationAmount = 0.2;

  /**
   * Approximate stopping distance at max velocity.
   *
   * Calculation is stopping distance assuming continuous and perfect motor command execution, plus some additional
   * buffer.
   *
   * This is not used for control - only for curvature evaluation.
   *
   * TODO: The additional buffer may need to be tuned.
   */
  const double kApproxMaxVelStoppingDist = ((std::pow(kMaxVel, 2)) / (abs(kMaxDecel) * 2)) + 0.3;

  /**
   * Default value for the additional length on top of the approximated stopping distance that a path must have to be
   * considered a reasonably open path. See open_free_path_len_threshold_.
   */
  const double kDefaultFreePathBufferLenThreshold = 2;

  /**
   * Default weight that clearance should have in the path scoring function used when there are no "reasonably open"
   * paths. See scoring_clearance_weight_.
   */
  const double kDefaultClearanceWeight = 0.005;

  /**
   * Default weight that curvature should have in the path scoring function used when there are no "reasonably open"
   * paths. See scoring_curvature_weight_.
   */
  const double kDefaultCurvatureWeight = -0.005;

  /**
   * ROS parameter name for setting the clearance weight for the path scoring function.
   */
  const std::string kScoringClearanceWeightParamName = "scoring_clearance_weight";

  /**
   * ROS parameter name for setting the curvature weight for the path scoring function.
   */
  const std::string kScoringCurvatureWeightParamName = "scoring_curvature_weight";

  /**
   * ROS parameter name for setting the buffer length (added onto the approximate stopping distance) used to compute
   * the threshold for a "reasonably open" path.
   */
  const std::string kOpenFreePathAfterStopDistThresholdParamName = "open_path_len_threshold";

  /**
   * ROS parameter name for setting the threshold for the clearance that a path needs to be considered "reasonably
   * open".
   */
  const std::string kOpenClearanceThresholdParamName = "open_path_clearance_threshold";

  /**
   * Curvatures to evaluate at each time step.
   *
   * Constant after object is constructed.
   */
  const std::vector<double> curvatures_to_evaluate_;

  /**
   * Number of timesteps to account for in latency compensation. (Should be actuation latency
   */
  const int kNumActLatencySteps = ceil(kActuationLatency / kLoopExecutionDelay);

  /**
   * Color for drawing car boundaries.
   */
  const uint32_t kCarBoundariesColor = 0x34b4eb;

  /**
   * Color for drawing safety margin boundaries.
   */
  const uint32_t kCarSafetyMarginColor = 0x34eb4c;

  /**
   * Color for drawing the predicted car boundaries (car boundaries after executing a path) when the path is not
   * reasonably open.
   */
  const uint32_t kPredictedCarBoundariesColor = 0xeb34d2;

  /**
   * Color for drawing the predicted car boundaries with the safety margin (after executing a path), when the path is
   * not reasonably open.
   */
  const uint32_t kPredictedCarSafetyMarginColor = 0xeb3443;

  /**
   * Color for drawing the predicted car boundaries when the path is reasonably open.
   */
  const uint32_t kPredictedOpenPathCarBoundariesColor = 0xecf542;

  /**
   * Color for drawing the predicted safety margin boundaries when the path is reasonably open.
   */
  const uint32_t kPredictedOpenPathSafetyMarginColor = 0x84f542;

  /**
   * Color of the cross for the carrot.
   */
  const uint32_t kCarrotColor = 0x7932a8;

  /**
   * Color to display the global plan and its nodes in.
   */
  const uint32_t kGlobalPlanColor = 0x435614;

  /**
   * Size of the cross for the global plan points.
   */
  const float kGlobalPlanCrossSize = 0.1;

  /**
   * Size of the cross for the carrot.
   */
  const float kCarrotCrossSize = 0.3;

  /**
   * Executed commands. This will have kNumActLatency steps in it and with the most recent command at index 0 and the
   * least recent command last.
   */
  std::vector<amrl_msgs::AckermannCurvatureDriveMsg> recent_executed_commands;

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // PointCloud from LaserScan
  std::vector<Eigen::Vector2f> cloud_;
  double scan_time_;
  
  // Add car body dimensions for free path lengths - Notation from Amanda's Solutions
  //TODO: verify that simulator values match reality and consider shrinking safety margin
  const float b = 0.324;
  const float l = 0.535;
  const float w = 0.281;
  const float m = 0.2; // Choosing a safety margin of 20 cm to be conservative
  const float kAxleToRearDist = 0.5 * (l - b);
  const float kAxleToFrontDist = l - kAxleToRearDist;

  // Constants for simpler calculations
  const float kLengthFromAxleToSafetyFront = m + 0.5*(l+b);
  const float kLengthFromBaseToSafetySide = 0.5 * w + m;

  /**
   * Amount to allow the car to deviate from the straight line distance between two nodes on planning graph. Must be at
   * least as large as the deviation required to execute the optimal curve between the two nodes, given their angles.
   *
   * Effectively draw a rectangle with each node at the center of opposite ends and the sides of the edges adjacent to
   * the ends with the nodes should be this distance from the node.
   */
  const float kLateralDeviationFromPlanAllowance = 1.25;

  /**
   * Maximum angular deviation from the plan. Does take angle along arcs between nodes into account.
   */
  const float kAngularDeviationFromPlanAllowance = M_PI_4;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  
  /**
   * Navigation graph that has only the grid_aligned nodes.
   */
  nav_graph::NavGraph permanent_navigation_graph_;

  /**
   * Navigation graph that has the start/goal nodes added to it. This should be replaced by a copy from the permanent
   * nav graph when a new goal is set.
   */
  nav_graph::NavGraph temp_node_nav_graph_;

  /**
   * Node for the goal.
   */
  nav_graph::NavGraphNode goal_node_;

  /**
   * Node for the start.
   */
  nav_graph::NavGraphNode start_node_;

  // Whether navigation graph is ready or not.
  bool is_nav_graph_ready_ = false;

  /**
   * File containing the map.
   */
  std::string map_file_;

  // Map of the environment.
  vector_map::VectorMap map_;

  /**
   * Inflated map of the environment.
   */
  vector_map::VectorMap inflated_map_;

  /**
   * True if we've just set a goal and need to replan. False, if we should continue working toward the last set goal.
   */
  bool new_goal_;

  /**
   * Weight for the clearance in the scoring function. Clearance is good so this should be a positive number.
   */
  double scoring_clearance_weight_;

  /**
   * Weight for the curvature in the scoring function. Because our goal is along the x-axis, and lower (absolute)
   * curvatures will bring us close to our goal faster, we want to minimize the curvature (therefore, this should be
   * negative).
   *
   * TODO: If we end up with goals not aligned with the x axis, we should consider positively weighting the dot product
   * of the vector to the goal location and the vector to the point along the curve that will get us closest to the
   * goal location.
   */
  double scoring_curvature_weight_;

  /**
   * Minimum clearance that a path must have to be considered "reasonably open". Must be nonnegative. This is applied
   * after the safety margin.
   */
  double open_clearance_threshold_;

  /**
   * Free path length that a path must have to be considered reasonably open. This is an approximate stopping distance
   * at max velocity plus some additional distance. This is because we would like to traverse at max velocity for
   * some time before having to stop in a "reasonably open" path.
   */
  double open_free_path_len_threshold_;

  /**
   * Global plan that we have left to execute. Should have nodes removed as we're close enough to them. Will be
   * replaced when we replan.
   */
  std::vector<nav_graph::NavGraphNode> global_plan_to_execute_;

  /**
   * Get the position of the base_link of the car relative to the current base_link position after executing the curvature for the given path length.
   *
   * @param curvature   Curvature
   * @param path_len    Path length
   *
   * @return Position relative to current position of the car after executing the command for curvature along the given path length.
   */
  std::pair<Eigen::Vector2f, double> getLocationAfterCurvatureExecution(const double &curvature, const double &path_len);

  /**
   * Transform the given location.
   *
   * Input location is in coordinate frame A. Transform gives the position of coordinate frame A in the desired output
   * frame.
   *
   * @param loc_to_transform    Location to transform.
   * @param transform_info      Location and angular offset of frame of the location to transform in the desired output
   *                            coordinate frame.
   *
   * @return Location in the output coordinate frame.
   */
  Eigen::Vector2f transformLocation(const Eigen::Vector2f &loc_to_transform,
      const std::pair<Eigen::Vector2f, double> &transform_info);

  /**
   * Draw the car's position at the end points of each of the given paths.
   *
   * @param free_path_len_and_clearance_by_curvature    Free path length and clearance (in that order) by the curvature
   *                                                    that they correspond to.
   */
  void drawCarPosAfterCurvesExecuted(
          const std::unordered_map<double, std::pair<double, double>> &free_path_len_and_clearance_by_curvature);

  /**
   * Check if the path with the given clearance and free path length is reasonably open.
   *
   * @param free_path_len   Free path length.
   * @param clearance       Clearance of the path.
   *
   * @return True if the path is reasonably open, false if not.
   */
  bool isPathReasonablyOpen(const double &free_path_len, const double &clearance);

  /**
   * Add display of the car dimensions assuming the origin of the car is at the given location to the visualization
   * message.
   *
   * @param car_origin_loc[in]  Location of the origin of the car.
   * @param car_color_loc[in]   Color to display the car boundaries in.
   * @param safety_color[in]    Color to display the safety margin boundaries in.
   * @param viz_msg[out]     Visualization message that should be updated with the car display
   */
  void addCarDimensionsAndSafetyMarginAtPosToVisMessage(const std::pair<Eigen::Vector2f, double> &car_origin_loc,
      const uint32_t &car_color_loc, const uint32_t &safety_color, amrl_msgs::VisualizationMsg &viz_msg);


  /**
   * Add components to the visualization message for seeing the car size and car+safety margin size.
   *
   * @param viz_msg[out] Visualization message to update. Should be the local viz msg.
   */
  void addCarDimensionsAndSafetyMarginToVisMessage(amrl_msgs::VisualizationMsg &viz_msg);

  /**
   * Compute the distance it will take to decelerate to 0 from the current velocity.
   *
   * @param velocity_to_decelerate_from Velocity to decelerate from.
   *
   * @return Distance it will take to reach zero velocity when decelerating at the max deceleration rate.
   */
  double computeDecelDistance(const double &velocity_to_decelerate_from);

  /**
   * Get the curvatures to evaluate and chose from.
   *
   * @return Curvatures to evaluate and consider for execution for the next time step.
   */
  std::vector<double> getCurvaturesToEvaluate();

  /**
   * Get the curvature to follow for the next timestep.
   *
   * @param curvature_and_obstacle_limitations_map  Map of curvature to a pair of the free path length and clearance for
   *                                                the curvature.
   * @param carrot                                  Contains the location that the car should try to get to with
   *                                                respect to the base_link frame.
   *
   * @return Pair of curvature to follow and distance for which to follow it.
   */
  std::pair<double, double> chooseCurvatureForNextTimestep(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map,
          const std::pair<Eigen::Vector2f, float> &carrot);

  /**
   * Get the curvature that would bring the car exactly to the given location.
   *
   * @param carrot_location Location to get to with the car (relative to base_link frame).
   *
   * @return Curvature that will bring the car to the carrot.
   */
  double getOptimalCurvatureForCarrot(const Eigen::Vector2f &carrot_location);

  /**
   * If there are several fairly open paths, we'll want to take the one that has the lowest curvature, because that
   * will allow us to most quickly reach the goal x meters in front of us. This function returns all curvatures with
   * "relatively open" paths (i.e. exceed some desired clearance threshold and some desired free path length
   * threshold).
   *
   * @param curvature_and_obstacle_limitations_map  Map of curvature to a pair of the free path length and clearance for
   *                                                the curvature.
   *
   * @return List of curvatures with relatively open paths.
   */
  std::vector<double> getCurvaturesWithReasonablyOpenPaths(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map);

  /**
   * Pick the curvature for the next timestep assuming there are no "reasonably open" paths. This takes into account
   * the curvature (which we want to make as close to the optimal curvature for the carrot as possible), as well as the
   * clearance (which we want to maximize) and the free path length (which we want to maximize).
   *
   * @param curvature_and_obstacle_limitations_map  Map of curvature to a pair of the free path length and clearance for
   *                                                the curvature.
   * @param carrot                                  Location to get to with the car (used to calculate optimal
   *                                                curvature).
   *
   * @return Best curvature.
   */
  double chooseCurvatureForNextTimestepNoOpenOptions(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map,
          const std::pair<Eigen::Vector2f, float> &carrot);

  /**
   * Score the curvature for the next timestep.
   *
   * This takes into account the curvature (the absolute value of which we want to minimize, because increased absolute
   * curvature increases how much we are deviating from the goal along the x axis), as well as the clearance (which we
   * want to maximize) and the free path length (which we want to maximize).
   *
   * @param curvature       Curvature (inv turning radius).
   * @param free_path_len   Free path length that can be followed along curvature (this is the obstacle-free path
   *                        length, not the path length that we should follow to get closest to our goal, which may be
   *                        less than the obstacle-free path length)
   * @param clearance       Minimum clearance to obstacles along this curvature.
   * @param best_curvature  Optimal curvature for getting to the carrot location.
   *
   * @return Score for the curvature.
   */
  double scoreCurvature(const double &curvature, const double &free_path_len, const double &clearance,
                        const double &best_curvature);

  /**
   * Get the free path length and clearance (distance to the closest obstacle) for each of the given curvatures.
   *
   * TODO: Bharath, feel free to replace this with an implementation that computes the free path lengths and clearances
   * in batch if you can reuse computations. Delete the single-curvature function if you end up doing multiple at once.
   * As long as this function's signature is consistent, that's fine.
   *
   * @param curvatures_to_evaluate Curvatures (inverse of radius of turning) to evaluate and path length for the
   * curvature that will bring the car to its closest point of appraoch to the carrot.
   *
   * @return Map with the curvature values as keys and pairs of the free path length and clearance (in that order) for
   * the respective curvature as values. Free path length will be at most the length to traverse for the closest point
   * of approach (i.e. there could be more free space along the curvature, but we're not evaluating it).
   */
  std::unordered_map<double, std::pair<double, double>> getFreePathLengthsAndClearances(
          const std::unordered_map<double, double> &curvatures_to_evaluate_with_free_path_len_to_closest_point_of_approach);

  /**
   * Get the free path length and clearance (distance to the closest obstacle) for the given
   * curvature.
   *
   * TODO: Bharath, implement this (or the batch version if you can reuse computations).
   *
   * @param curvature Curvature (inv of radius of turning) to find the free path length and clearance for.
   *
   * @return Pair with the first entry as the free path length and second entry as the clearance.
   */
  std::pair<double, double> getFreePathLengthAndClearance(const double &curvature, const double &free_path_len_to_closest_point_of_approach);

  /**
   * Command the drive to traverse the given distance along this curvature. Should only issue one command (we may
   * issue a different curvature at the next time step).
   *
   * TODO: Kunal, implement this. If there is some way that this would fail, feel free to add a boolean or int return
   * code to indicate failure.
   *
   * TODO: Should this be open-loop (we just estimate how far we've gone) or semi-closed loop (use odom readings to
   * estimate how far we've driven).
   *
   * @param distance    Distance to travel. This is mainly provided so that we know if we have to start decelerating
   *                    or not.
   * @param curvature   Curvature (inv radius of turning) to execute.
   */
  void executeTimeOptimalControl(const double &distance, const double &curvature);

  /**
   * Construct the curvatures to evaluate. Since this list will not change throughout the execution of the program,
   * this will be called on start up to initialize a variable that will contain the curvatures to evaluate.
   *
   * @return List of curvatures to evaluate.
   */
  std::vector<double> constructCurvaturesToEvaluate();

  /**
   * For each curvature, get the path length along the curvature that will get the car closest to the goal (path length
   * to closest point of approach).
   *
   * @param carrot      Location to reach in the base_link frame.
   * @param curvatures  Curvatures to get the path length to the closest point of approach for.
   *
   * @return Map of the curvature to the path length that would bring the car to the closest point of approach to the
   * goal.
   */
  std::unordered_map<double, double> getPathLengthForClosestPointOfApproach(
            const Eigen::Vector2f &carrot, const std::vector<double> &curvatures);

  /**
   * Get a target for local planning.
   *
   * @return Local planning target.
   */
  std::pair<Eigen::Vector2f, float> getCarrot();

  /**
   * Run obstacle avoidance with the goal of getting to the carrot.
   *
   * @param carrot Target to reach with local planning.
   */
  void runObstacleAvoidance(const std::pair<Eigen::Vector2f, float> &carrot);

  /**
   * Display the global path.
   */
  void displayGlobalPath();

  /**
   * Inflate the map by extending each line on both sides of the line, and put lines paralleling each line.
   *
   * @param original_map[in]    Original map.
   * @param new_map[out]        Map with additional lines and the original lines extended.
   */
  void computeInflatedMap(const vector_map::VectorMap &original_map, vector_map::VectorMap &new_map);

  /**
   * True if the car is roughly along the path between the two nodes, false if it is outside these nodes. Car can
   * deviate somewhat in distance from the line connecting the node and the curve, but if it is too far, this returns
   * false.
   *
   * @param node_1 Node that the car will be at or past if this returns true.
   * @param node_2 Node that the car will be immediately preceding if this returns true.
   *
   * @return True if the car is close enough to the path between node 1 and node 2 to be considered between the nodes.
   */
  bool isCarInBetweenNodes(const nav_graph::NavGraphNode &node_1, const nav_graph::NavGraphNode &node_2);

  /**
   * Check if the plan is still valid and remove any nodes that we've driven past (except the one immediately preceding the bot).
   *
   * @return True if the plan is still valid, false if we need to replan.
   */
  bool planStillValid();

  /**
   * Update nav complete to indicate if we've reached our goal.
   */
  void ReachedGoal();
  /*
   * creates navigation graph
   */
  void createNavGraph();

  /**
   * Draw the carrot (in the base_link frame).
   *
   * @param carrot Carrot that car is trying to get to as an intermediate step toward the goal.
   */
  void drawCarrot(const Eigen::Vector2f &carrot);
};
}  // namespace navigation

#endif  // NAVIGATION_H
