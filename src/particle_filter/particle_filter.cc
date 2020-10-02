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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter(ros::NodeHandle* n) :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {
    n->param(kNumParticlesParamName, num_particles_, kDefaultNumParticles);
    n->param(kInitialXStdDevParamName, initial_x_stddev_, kDefaultInitialXStdDev);
    n->param(kInitialYStdDevParamName, initial_y_stddev_, kDefaultInitialYStdDev);
    n->param(kInitialThetaStdDevParamName, initial_theta_stddev_, kDefaultInitialThetaStdDev);

    n->param(kMotionModelAlpha1ParamName, motion_model_alpha_1, kDefaultMotionModelAlpha1);
    n->param(kMotionModelAlpha2ParamName, motion_model_alpha_2, kDefaultMotionModelAlpha2);
    n->param(kMotionModelAlpha3ParamName, motion_model_alpha_3, kDefaultMotionModelAlpha3);
    n->param(kMotionModelAlpha4ParamName, motion_model_alpha_4, kDefaultMotionModelAlpha4);

    ROS_INFO_STREAM("Number of particles: " << num_particles_);
    ROS_INFO_STREAM("Std dev for initial pose x, y, and theta " << initial_x_stddev_ << ", " << initial_y_stddev_
            << ", " << initial_theta_stddev_);
    ROS_INFO_STREAM("Motion model parameters (1-4): " << motion_model_alpha_1 << ", " << motion_model_alpha_2
            << ", " << motion_model_alpha_3 << ", " << motion_model_alpha_4);
}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  if (odom_initialized_) {
      // Implementing odometry based motion model found on pg. 136 of Probabilistic Robotics book
      float prev_odom_x = prev_odom_loc_.x();
      float curr_odom_x = odom_loc.x();

      float prev_odom_y = prev_odom_loc_.y();
      float curr_odom_y = odom_loc.y();

      double delta_rot_1 = atan2(curr_odom_y - prev_odom_y, curr_odom_x - prev_odom_x) - prev_odom_angle_;
      double delta_trans = (odom_loc - prev_odom_loc_).norm();
      double delta_rot_2 = odom_angle - prev_odom_angle_ - delta_rot_1;

      for (uint32_t i = 0; i < particles_.size(); i++) {

          // TODO ppts of this concept don't square delta_rot_1, delta_trans, or delta_rot_2. Should we?
          double delta_rot_1_std_dev =
                  sqrt((motion_model_alpha_1 * pow(delta_rot_1, 2)) + (motion_model_alpha_2 * pow(delta_trans, 2)));
          double delta_trans_std_dev = sqrt((motion_model_alpha_3 * pow(delta_trans, 2)) +
                  (motion_model_alpha_4 * (pow(delta_rot_1, 2) + pow(delta_rot_2, 2))));
          double delta_rot_2_std_dev =
                  sqrt((motion_model_alpha_1 * pow(delta_rot_2, 2)) + (motion_model_alpha_2 * pow(delta_trans, 2)));

          double delta_rot_1_noise = rng_.Gaussian(0.0, sqrt(delta_rot_1_std_dev));
          double delta_trans_noise = rng_.Gaussian(0.0, sqrt(delta_trans_std_dev));
          double delta_rot_2_noise = rng_.Gaussian(0.0, sqrt(delta_rot_2_std_dev));

          double delta_rot_1_hat = delta_rot_1 - delta_rot_1_noise;
          double delta_trans_hat = delta_trans - delta_trans_noise;
          double delta_rot_2_hat = delta_rot_2 - delta_rot_2_noise;

          Particle curr_particle = particles_[i];
          float curr_x = curr_particle.loc.x();
          float curr_y = curr_particle.loc.y();
          float curr_theta = curr_particle.angle;

          float new_x = curr_x + (delta_trans_hat * cos(curr_theta + delta_rot_1_hat));
          float new_y = curr_y + (delta_trans_hat * sin(curr_theta + delta_rot_1_hat));
          float new_theta = curr_theta + delta_rot_1_hat + delta_rot_2_hat;

          particles_[i].angle = math_util::AngleMod(new_theta);
          particles_[i].loc = Vector2f(new_x, new_y);
      }
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
  odom_initialized_ = true;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  map_.Load("maps/" + map_file + ".txt");
  odom_initialized_ = false;
  particles_.clear();
  for (int i = 0; i < num_particles_; i++) {
      double weight = 1.0 / num_particles_;
      Particle particle;
      float x = rng_.Gaussian(loc.x(), initial_x_stddev_);
      float y = rng_.Gaussian(loc.y(), initial_y_stddev_);
      float noisy_angle = rng_.Gaussian(angle, initial_theta_stddev_);
      particle.loc = {x, y};
      particle.angle = math_util::AngleMod(noisy_angle);
      particle.weight = weight;
      particles_.emplace_back(particle);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
