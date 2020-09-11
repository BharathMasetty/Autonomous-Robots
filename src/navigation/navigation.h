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

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

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
   * TODO: Amanda, implement this.
   * TODO: May also need to add some idea of goal as a parameter.
   *
   * @param curvature_and_obstacle_limitations_map  Map of curvature to a pair of the free path length and clearance for
   *                                                the curvature.
   *
   * @return Pair of curvature to follow and distance for which to follow it.
   */
  std::pair<double, double> chooseCurvatureForNextTimestep(
          std::unordered_map<double, std::pair<double, double>> &curvature_and_obstacle_limitations_map);

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
};

}  // namespace navigation

#endif  // NAVIGATION_H
