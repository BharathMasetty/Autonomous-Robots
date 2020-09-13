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

#include <vector>
#include <unordered_map>

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
   * Amount of time between each loop execution.
   */
  const double kLoopExecutionDelay = (1.0 / 20.0);

  /**
   * Maximum curvature (assumed to be the same on both sides of the car, so c_min = -1 *c_max).
   *
   * TODO Investigate this value.
   */
  const double kMaxCurvature = 1.0;

  /**
   * Number of curvatures to evaluate. These will be evenly spaced between c_min and c_max
   * (see kMaxCurvature).
   *
   * This should be a positive odd number >= 3, so we can evaluate straight as an option.
   */
  const int kNumCurvaturesToEval = 41;

  /**
   * X position of intermediate goal in base_link frame. Robot should try to get to this
   * point for each iteration.
   */
  const double kIntermediateGoalX = 4.0;

  /**
   * Default value for the clearance that a path must have to be considered "reasonably open".
   */
  const double kDefaultDesiredClearance = 0.2;

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
  const double kDefaultFreePathBufferLenThreshold = 1.5;

  /**
   * Default weight that clearance should have in the path scoring function used when there are no "reasonably open"
   * paths. See scoring_clearance_weight_.
   */
  const double kDefaultClearanceWeight = 0.1;

  /**
   * Default weight that curvature should have in the path scoring function used when there are no "reasonably open"
   * paths. See scoring_curvature_weight_.
   */
  const double kDefaultCurvatureWeight = -0.3;

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
  //TODO: get the actual values from car considering safety margin
  const float b = 0.1;
  const float l = 0.15;
  const float w = 0.1;
  const float m = 0.05; // Choosing a safety margin of 5 cm for now.
  // Constants for simpler calculations

  const float kLengthFromAxleToSafetyFront = m + 0.5*(l+b);
  const float kLengthFromBaseToSafetySide = 0.5 * w + m;
   


  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

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
   *
   * @return Pair of curvature to follow and distance for which to follow it.
   */
  std::pair<double, double> chooseCurvatureForNextTimestep(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map);

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
   * the curvature (the absolute value of which we want to minimize, because increased absolute curvature increases how
   * much we are deviating from the goal along the x axis), as well as the clearance (which we want to maximize) and
   * the free path length (which we want to maximize).
   *
   * @param curvature_and_obstacle_limitations_map  Map of curvature to a pair of the free path length and clearance for
   *                                                the curvature.
   *
   * @return Best curvature.
   */
  double chooseCurvatureForNextTimestepNoOpenOptions(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map);

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
   *
   * @return Score for the curvature.
   */
  double scoreCurvature(const double &curvature, const double &free_path_len, const double &clearance);

  /**
   * Get the free path length and clearance (distance to the closest obstacle) for each of the given curvatures.
   *
   * TODO: Bharath, feel free to replace this with an implementation that computes the free path lengths and clearances
   * in batch if you can reuse computations. Delete the single-curvature function if you end up doing multiple at once.
   * As long as this function's signature is consistent, that's fine.
   *
   * @param curvatures_to_evaluate Curvatures (inverse of radius of turning) to evaluate.
   *
   * @return Map with the curvature values as keys and pairs of the free path length and clearance (in that order) for
   * the respective curvature as values.
   */
  std::unordered_map<double, std::pair<double, double>> getFreePathLengthsAndClearances(
          const std::vector<double> &curvatures_to_evaluate);

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
  std::pair<double, double> getFreePathLengthAndClearance(const double &curvature);
  

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
   * Get the free path length along the arc defined by the given curvature, that if traversed, would bring the car
   * closest to the goal.
   *
   * Assumes that the goal is on the x-axis. If we want to expand this to arbitrary goals, we'll need to use the cosine
   * rule.
   *
   * @param goal_in_bl_frame_x      X coordinate of the goal. Assumed to be positive.
   * @param curvature               Curvature defining arc to traverse.
   * @param obstacle_free_path_len  Path length along arc for which we will not hit any obstacles.
   *
   * @return Free path length along arc that will bring the car closest to the goal position without hitting obstacles.
   */
  double getFreePathLengthToClosestPointOfApproach(double goal_in_bl_frame_x, double curvature,
                                                   double obstacle_free_path_len);
};

}  // namespace navigation

#endif  // NAVIGATION_H
