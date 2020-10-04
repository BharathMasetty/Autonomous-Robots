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
                              std::vector<Eigen::Vector2f>* scan,
			      std::vector<float>* ranges);

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
   * ROS parameter name for the first motion model parameter (used in standard deviation of rotation).
   */
  const std::string kMotionModelAlpha1ParamName = "motion_model_alpha_1";

  /**
   * ROS parameter name for the second motion model parameter (used in standard deviation of rotation).
   */
  const std::string kMotionModelAlpha2ParamName = "motion_model_alpha_2";

  /**
   * ROS parameter name for the third motion model parameter (used in standard deviation of translation).
   */
  const std::string kMotionModelAlpha3ParamName = "motion_model_alpha_3";

  /**
   * ROS parameter name for the fourth motion model parameter (used in standard deviation of translation).
   */
  const std::string kMotionModelAlpha4ParamName = "motion_model_alpha_4";
 
  /**
  * ROS parameter name for gamma (observation model) in update step
  */
 const std::string kObsGammaParamName = "gamma";
 
 /**
  * ROS parameter name for standard deviation of observation model
  */
 const std::string kObsStdDevSquaredParamName = "observation_std_dev";
 /**
  * ROS parameter name for d_short in observation model
  */
 const std::string kObsDshortParamName = "observation_d_short";
 /*
  *ROS parameter name for d_long in observation model
  */
 const std::string kObsDlongParamName = "observation_d_long";
 /*
  *ROS paramter name for s_min in observation model
  */
 const std::string kObsSminParamName = "observation_s_min";
 /*
  *ROS parameter name for s_max in observation model
  */
  const std::string kObsSmaxParamName = "observation_s_max";
 /*
  * ROS parameter name for D - distance between 2 update calls
  */
  const std::string kObsDParamName = "observation_d";
  /*
   * ROS Parameter name for K - number of updates between two resample calls
   */
  const std::string kObsKParamName = "observation_k";

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

  /**
   * Default value for the first motion model parameter (used in standard deviation of rotation).
   */
  const double kDefaultMotionModelAlpha1 = 0.1;

  /**
   * Default value for the second motion model parameter (used in standard deviation of rotation).
   */
  const double kDefaultMotionModelAlpha2 = 0.1;

  /**
   * Default value for the third motion model parameter (used in standard deviation of translation).
   */
  const double kDefaultMotionModelAlpha3 = 0.1;

  /**
   * Default value for the fourth motion model parameter (used in standard deviation of translation).
   */
  const double kDefaultMotionModelAlpha4 = 0.1;

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

  /*
   *  Returns the closest point and intersection and distance to it along a given laser line.
   */
  std::pair<Eigen::Vector2f, float> GetIntersectionPoint(const Eigen::Vector2f LidarLoc, 
		  					 const float laserAngle, 
							 const float angle, 
							 float range_min, 
							 float range_max);

  /*
   * Traslation between Lidar and baseLink
   * This distance is along x axis of the base link (Lidar Loc wrt base link is (0.2, 0))
   */
  const float kDisplacementFromBaseToLidar = 0.2;

  /*
   * Gamma: correlation parameter for observation model in update step
   * gamma = 1 => No correlation among cloud points.
   * gamma = 1/n where n is number of cloud points => Full correlation.
   */
  const double kDefaultGamma = 1; 
  double gamma_; 
  /*
   *squared standard deviation of the observation model
   */
  const float kDefaultSquaredLaserStdDev = 1.0; 
  float squared_laser_stddev_;
  /*
   * squared d_short for robust observation liklihood 
   */ 
  const float kDefaultDshort = 0.3;
  float d_short_;
  float squared_d_short_ = std::pow(d_short_,2); 
  /*
   * Fixed obs probability for d_short
   */
  float d_short_prob_ = std::exp(-squared_d_short_/squared_laser_stddev_); 
  
  /*
   * Squared d_long for robust observation liklihood
   */ 
  const float kDefaultDlong = 0.3;
  float d_long_;
  float squared_d_long_ = std::pow(d_long_,2);
  
  /*
   * Fixed Obs Probability for d_long
   */
  float d_long_prob_ = std::exp(-squared_d_long_/squared_laser_stddev_);
  
 /*
  * s_min for normalizing  Observation liklihood model (for making area under pdf = 1)
  */  
  float s_min_;
  const float kDefaultSmin = 1.0;
  
  /*
   * s_max for normalizing observation liklihood model (for making aread under pdf = 1)
   */
  float s_max_;
  const float kDefaultSmax = 1.0;
  
  /*
   * D = distance the car has to travel between two update steps 
   * Setting it to a default value of 10 cm
   */
   const float kDefaultObsD = 0.10; 
   float obs_d_;
   
   /*
    * K = Number of update steps before resampling 
    * setting to a default value of 10
    */
   const int kDefaultObsK = 10;
   int obs_k_;


  /**
   * First motion model parameter (used in standard deviation of rotation).
   */
  double motion_model_alpha_1;

  /**
   * Second motion model parameter (used in standard deviation of rotation).
   */
  double motion_model_alpha_2;

  /**
   * Third motion model parameter (used in standard deviation of translation).
   */
  double motion_model_alpha_3;

  /**
   * Fourth motion model parameter (used in standard deviation of translation).
   */
  double motion_model_alpha_4;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
