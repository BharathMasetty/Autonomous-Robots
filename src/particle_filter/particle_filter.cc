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

//DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter(ros::NodeHandle* n) :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {

    loadParams(n);

    dispFromLastUpdate_ = 0;
    lastUpdateBestParticleIndex_ = -1;
}

void ParticleFilter::loadParams(ros::NodeHandle *n) {
    ROS_INFO_STREAM("Reloading parameters");

    // Initialization Parameters
    n->param(kNumParticlesParamName, num_particles_, kDefaultNumParticles);
    n->param(kInitialXStdDevParamName, initial_x_stddev_, kDefaultInitialXStdDev);
    n->param(kInitialYStdDevParamName, initial_y_stddev_, kDefaultInitialYStdDev);
    n->param(kInitialThetaStdDevParamName, initial_theta_stddev_, kDefaultInitialThetaStdDev);

    // Motion model parameters
    n->param(kMotionModelAlpha1ParamName, motion_model_alpha_1_, kDefaultMotionModelAlpha1);
    n->param(kMotionModelAlpha2ParamName, motion_model_alpha_2_, kDefaultMotionModelAlpha2);
    n->param(kMotionModelAlpha3ParamName, motion_model_alpha_3_, kDefaultMotionModelAlpha3);
    n->param(kMotionModelAlpha4ParamName, motion_model_alpha_4_, kDefaultMotionModelAlpha4);

    // Observation Model Parameters
    n->param(kObsGammaParamName, gamma_, kDefaultGamma);
    n->param(kObsStdDevSquaredParamName, squared_laser_stddev_, kDefaultSquaredLaserStdDev);
    n->param(kObsDshortParamName, d_short_, kDefaultDshort);
    n->param(kObsDlongParamName, d_long_, kDefaultDlong);
    n->param(kObsDParamName, obs_d_, kDefaultObsD);
    n->param(kObsKParamName, obs_k_, kDefaultObsK);

    n->param(kGetLocationAveragingDistParamName, get_loc_averaging_dist_, kDefaultGetLocAveragingDist);

    ROS_INFO_STREAM("Number of particles: " << num_particles_);
    ROS_INFO_STREAM("Std dev for initial pose x, y, and theta " << initial_x_stddev_ << ", " << initial_y_stddev_
                                                                << ", " << initial_theta_stddev_);
    ROS_INFO_STREAM("Motion model parameters (1-4): " << motion_model_alpha_1_ << ", " << motion_model_alpha_2_
                                                      << ", " << motion_model_alpha_3_ << ", " << motion_model_alpha_4_);
    ROS_INFO_STREAM("Observation Model Parameters: gamma: " << gamma_ << " square stddev-obs liklihood: " << squared_laser_stddev_);
    ROS_INFO_STREAM("Distance Between two update calls : " << obs_d_);
    ROS_INFO_STREAM("Updates between two resample calls: "<< obs_k_);

    squared_d_short_ = std::pow(d_short_, 2);
    d_short_log_prob_ = -squared_d_short_ / squared_laser_stddev_;
    squared_d_long_ = std::pow(d_long_, 2);
    d_long_log_prob_ = -squared_d_long_/squared_laser_stddev_;
}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

std::pair<Eigen::Vector2f, float> ParticleFilter::GetIntersectionPoint(const Vector2f &LidarLoc,
                                                                       const float &laserAngle,
                                                                       const float &angle,
                                                                       const float &range_min,
                                                                       const float &range_max) {

    // Laser Line start and end wrt map frame
    Vector2f startPoint(LidarLoc.x(), LidarLoc.y());
    Vector2f endPoint(LidarLoc.x() + range_max*cos(laserAngle + angle), LidarLoc.y() + range_max*sin(laserAngle + angle));

    // Laser Line
    line2f LaserLine(startPoint.x(), startPoint.y(), endPoint.x(), endPoint.y());

    // checking for intersection with map lines
    float currBestRange = range_max;
    Vector2f bestIntersection = endPoint;
    for (size_t i = 0; i < map_.lines.size(); ++i){
        const line2f map_line = map_.lines[i];
        Vector2f intersectionPoint;
        bool intersects = map_line.Intersection(LaserLine, &intersectionPoint);
        if (intersects) {
            float curr_range = (intersectionPoint - LidarLoc).norm();
            if (curr_range <= currBestRange) {
                if (curr_range < range_min) {
                    currBestRange = range_min;
                    bestIntersection = Vector2f(LidarLoc.x() + range_min * cos(laserAngle + angle), LidarLoc.y() + range_min * sin(laserAngle + angle));
                } else {
                    currBestRange = curr_range;
                    bestIntersection = intersectionPoint;
                }
            }
        }
    }

    return std::make_pair(bestIntersection, currBestRange);
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr,
                                            vector<float>* ranges_ptr) {
  // Lidar Location wrt Map Frame
  Vector2f LidarLoc(loc.x() + kDisplacementFromBaseToLidar*cos(angle), loc.y() + kDisplacementFromBaseToLidar*sin(angle));
  
  vector<Vector2f>& scan = *scan_ptr;
  vector<float>& ranges = *ranges_ptr;
  scan.resize(num_ranges);
  ranges.resize(num_ranges);
  const float angleIncrement = (angle_max - angle_min)/(num_ranges-1);
  // Predictions for each scan direction 
  for (int i=0; i< num_ranges; i += laser_scan_keep_ratio_) {

    float currLaserAngle = angle_min + i*angleIncrement;
    std::pair<Vector2f, float> cloudPointInfo = GetIntersectionPoint(LidarLoc, currLaserAngle, angle, range_min, range_max);
    scan[i] = cloudPointInfo.first;
     ranges[i] = cloudPointInfo.second;
  }
}

int ParticleFilter::getScanReductionIncrement() {
    return laser_scan_keep_ratio_;
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                  Particle* p_ptr) {
  
  Particle& p = *p_ptr; 
  vector<Vector2f> predictedPointCloud;
  vector<float> predictedRanges;
  const int num_ranges = ranges.size();
  GetPredictedPointCloud(p.loc, p.angle, num_ranges, range_min, range_max, angle_min, angle_max, &predictedPointCloud, &predictedRanges);
  
  float currObservationLogProb = 0; 
  for (int i = 0; i < num_ranges; i+= laser_scan_keep_ratio_){
      float diff = predictedRanges[i] - ranges[i];
      float squaredDiff = std::pow(diff, 2);

      if ((ranges[i] < range_min) || (ranges[i] > range_max)) {
          // Note: while this should be impossible (probability = 0), not going to change
          // the probability because then every single particle will have 0 probability. Instead, just ignoring this
          // laser reading
          ROS_ERROR_STREAM("Impossible reading: " << ranges[i]);
      } else if (ranges[i] < (predictedRanges[i] - d_short_)) {
          currObservationLogProb += d_short_log_prob_;
      } else if (ranges[i] > (predictedRanges[i] + d_long_)) {
          currObservationLogProb += d_long_log_prob_;
      } else {
          currObservationLogProb += -squaredDiff/squared_laser_stddev_;
      }
  }
  
  // accounting for correlation in the joint probability
  p.weight += gamma_*currObservationLogProb;
}

void ParticleFilter::Resample(const std::vector<double>& normWeightProbs) {

    double weight_sum = 0;

  // Initialize the D for calculating range
  std::vector<double> particles_weight_new;
  particles_weight_new.reserve(particles_.size());   //creating an empty array for calculating Weight*D
  std::vector<Particle> new_particles = {};                 //initialising a vector for particles
  double prev_particle_range_upper_bound = 0;
  for (size_t i = 0; i < particles_.size(); i++) {
      weight_sum += normWeightProbs[i];
      particles_weight_new[i] = normWeightProbs[i] + prev_particle_range_upper_bound;
      prev_particle_range_upper_bound = particles_weight_new[i];
  }

  //this loop will run for the given number of particles for resampling with unweighing
  float low_var_sampl_inc = weight_sum / particles_.size();
  float random_num = rng_.UniformRandom(0, weight_sum);

  size_t particle_search_index = 0;
  size_t best_weight_index = 0;
  for (size_t i =0; i<particles_.size(); i++) {

      while (random_num > particles_weight_new[particle_search_index]) {
          particle_search_index++;
      }
      new_particles.push_back(particles_[particle_search_index]);
      if (new_particles[i].weight > new_particles[best_weight_index].weight) {
          best_weight_index = i;
      }
      random_num += low_var_sampl_inc;
      if (random_num > weight_sum) {
          particle_search_index = 0;
          random_num -= weight_sum;
      }
  }
  lastUpdateBestParticleIndex_ = best_weight_index;
  particles_ = new_particles;
  //initialize all particle weights to be 1/N
  for (size_t i = 0; i<particles_.size(); i++){
      particles_[i].weight = log(1.0 /num_particles_);
  }
}


void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {

    if (particles_.empty()) {
        return;
    }

     /*
      * 1) Check if the car as traveled some distance
      * 2) Update if needed
      * 3) Check resampling
      * 4) Resample if needed  -- This resets the particle weights
      */

     int bestParticleIndex = 0;
     double bestWeight = -std::numeric_limits<double>::infinity();

     // Checking if we want to update or not
     if (dispFromLastUpdate_ >= obs_d_) {

         dispFromLastUpdate_ = 0;
         for (size_t i=0; i<particles_.size(); i++){
             Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
             if (particles_[i].weight >= bestWeight) {
                 bestWeight = particles_[i].weight;
                 bestParticleIndex = i;
             }
         }

         lastUpdateBestParticleIndex_ = bestParticleIndex;
         // Counter for updates done without resampling
         numUpdatesFromLastResample_ += 1;

         // Checking of we want to resample of not
         if (numUpdatesFromLastResample_ >= obs_k_){

            //Normalize Log probs
            std::vector<double> normalizedProbWeights;
            for (size_t i=0; i<= particles_.size(); i++) {
                normalizedProbWeights.push_back(getScaledWeight(particles_[i].weight,
                                                                particles_[bestParticleIndex].weight));
            }

            //Resample
            Resample(normalizedProbWeights);
            numUpdatesFromLastResample_ = 0;
        }
     }
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  if (odom_initialized_) {
      double delta_trans = (odom_loc - prev_odom_loc_).norm();
      float dir = 1.0;

      // checking if car is moving back or front
      Vector2f unrotated_disp_wrt_prev_frame = odom_loc - prev_odom_loc_;
      Eigen::Rotation2Df rotate(-1*prev_odom_angle_);
      Vector2f rotated_disp_wrt_prev_frame = rotate * unrotated_disp_wrt_prev_frame;
      if (rotated_disp_wrt_prev_frame.x() < 0){ 
	      dir *= -1.0;
      }

      if ((delta_trans != 0) || ((odom_angle - prev_odom_angle_) != 0)) {
          
	  // summation of absolute displacements for update step
	  dispFromLastUpdate_ += delta_trans;
	  
	  // adjusting change in displacement to direction
	  delta_trans *= dir;

          // Implementing odometry based motion model found on pg. 136 of Probabilistic Robotics book
          float prev_odom_x = prev_odom_loc_.x();
          float curr_odom_x = odom_loc.x();

          float prev_odom_y = prev_odom_loc_.y();
          float curr_odom_y = odom_loc.y();
	  float deltay = dir*(curr_odom_y - prev_odom_y);
	  float deltax = dir*(curr_odom_x - prev_odom_x);
          
	  double delta_rot_1 = atan2(deltay, deltax) - prev_odom_angle_;
	  double delta_rot_2 = odom_angle - prev_odom_angle_ - delta_rot_1;
	  
          // TODO ppts of this concept don't square delta_rot_1, delta_trans, or delta_rot_2. Should we?
          double delta_rot_1_std_dev =
                  sqrt((motion_model_alpha_1_ * pow(delta_rot_1, 2)) + (motion_model_alpha_2_ * pow(delta_trans, 2)));
          double delta_trans_std_dev = sqrt((motion_model_alpha_3_ * pow(delta_trans, 2)) +
                  (motion_model_alpha_4_ * (pow(delta_rot_1, 2) + pow(delta_rot_2, 2))));
          double delta_rot_2_std_dev =
                  sqrt((motion_model_alpha_1_ * pow(delta_rot_2, 2)) + (motion_model_alpha_2_ * pow(delta_trans, 2)));

          for (size_t i = 0; i < particles_.size(); i++) {

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
  dispFromLastUpdate_ = 0;
  lastUpdateBestParticleIndex_ = rng_.RandomInt(0, num_particles_);
  particles_.clear();
  for (int i = 0; i < num_particles_; i++) {
      double weight = std::log(1.0 / num_particles_);
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

    // Default location when there are no particles
    if(particles_.empty()){
        loc = Vector2f(0,0);
        angle = 0;
        return;
    }

    std::pair<Vector2f, float> smoothed_best_loc = getWeightedAverageRobotLocation(lastUpdateBestParticleIndex_);

    loc = smoothed_best_loc.first;
    angle = smoothed_best_loc.second;
}


std::pair<Eigen::Vector2f, float> ParticleFilter::getWeightedAverageRobotLocation(
            const size_t &highest_weight_particle_index) const {
    double x_weighted_avg = 0;
    double y_weighted_avg = 0;

    // Averaging angle by converting each angle to a point on the unit circle and then averaging those points, and
    // converting the resultant point back to polar coordinates
    // See http://en.wikipedia.org/wiki/Mean_of_circular_quantities
    double angle_x_weighted_avg = 0;
    double angle_y_weighted_avg = 0;
    double weight_sum = 0;

    Particle best_particle = particles_[highest_weight_particle_index];
    for (const Particle &particle : particles_) {
        if ((best_particle.loc - particle.loc).norm() <= get_loc_averaging_dist_) {
            double weight = getScaledWeight(particle.weight, best_particle.weight);
            weight_sum += weight;
            x_weighted_avg += weight * particle.loc.x();
            y_weighted_avg += weight * particle.loc.y();
            angle_x_weighted_avg += weight * cos(particle.angle);
            angle_y_weighted_avg += weight * sin(particle.angle);
        }
    }

    x_weighted_avg /= weight_sum;
    y_weighted_avg /= weight_sum;
    angle_x_weighted_avg /= weight_sum;
    angle_y_weighted_avg /= weight_sum;

    double angle = atan2(angle_y_weighted_avg, angle_x_weighted_avg);
    return std::make_pair(Vector2f(x_weighted_avg, y_weighted_avg), angle);
}

double ParticleFilter::getScaledWeight(const double &particle_log_likelihood,
                                       const double &max_particle_log_likelihood) const {
    return std::exp(particle_log_likelihood - max_particle_log_likelihood);
}
}  // namespace particle_filter
