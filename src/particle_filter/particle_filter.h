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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
  ParticleFilter(ros::NodeHandle* n);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

 private:

  /**
   * ROS parameter name for the number of particles.
   */
  const std::string kNumParticlesParamName = "num_particles";

  /**
   * ROS parameter name for the standard deviation to use when setting the x component of the initial set of particles.
   */
  const std::string kInitialXStdDevParamName = "initial_x_stddev";

  /**
   * ROS parameter name for the standard deviation to use when setting the y component of the initial set of particles.
   */
  const std::string kInitialYStdDevParamName = "initial_y_stddev";

  /**
   * ROS parameter name for the standard deviation to use when setting the theta component of the initial set of
   * particles.
   */
  const std::string kInitialThetaStdDevParamName = "initial_theta_stddev";

  /**
   * Default number of particles.
   */
  const int kDefaultNumParticles = 100;

  /**
   * Default value for the standard deviation to use when setting the x component of the initial particles.
   */
  const double kDefaultInitialXStdDev = 1.0;

  /**
   * Default value for the standard deviation to use when setting the y component of the initial particles.
   */
  const double kDefaultInitialYStdDev = 1.0;

  /**
   * Default value for the standard deviation to use when setting the theta component of the initial particles.
   */
  const double kDefaultInitialThetaStdDev = 0.3;

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  /**
   * Number of particles to use.
   */
  int num_particles_;

  /**
   * Standard deviation to use when setting the x component of the initial particles.
   */
  double initial_x_stddev_;

  /**
   * Standard deviation to use when setting the y component of the initial particles.
   */
  double initial_y_stddev_;

  /**
   * Standard deviation to use when setting the theta component of the initial particles.
   */
  double initial_theta_stddev_;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
