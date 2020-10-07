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
    n->param(kObsSminParamName, s_min_, kDefaultSmin);
    n->param(kObsSmaxParamName, s_max_, kDefaultSmax);
    n->param(kObsDParamName, obs_d_, kDefaultObsD);
    n->param(kObsKParamName, obs_k_, kDefaultObsK);

    ROS_INFO_STREAM("Number of particles: " << num_particles_);
    ROS_INFO_STREAM("Std dev for initial pose x, y, and theta " << initial_x_stddev_ << ", " << initial_y_stddev_
            << ", " << initial_theta_stddev_);
    ROS_INFO_STREAM("Motion model parameters (1-4): " << motion_model_alpha_1_ << ", " << motion_model_alpha_2_
            << ", " << motion_model_alpha_3_ << ", " << motion_model_alpha_4_);
    ROS_INFO_STREAM("Observation Model Parameters: gamma: " << gamma_ << " square stddev-obs liklihood: " << squared_laser_stddev_);
    ROS_INFO_STREAM("Distance Between two update calls : " << obs_d_);
    ROS_INFO_STREAM("Updates between two resample calls: "<< obs_k_);

    d_short_log_prob_ = -squared_d_short_ / squared_laser_stddev_;
    squared_d_short_ = std::pow(d_short_, 2);
    squared_d_long_ = std::pow(d_long_, 2);
    d_long_log_prob_ = -squared_d_long_/squared_laser_stddev_;

    dispFromLastUpdate_ = 0;
    lastUpdateBestParticleIndex_ = -1;
    justResampled_ = false;
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
    Vector2f startPoint(LidarLoc.x() + range_min*cos(laserAngle + angle), LidarLoc.y() + range_min*sin(laserAngle + angle));
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
            if (curr_range <= currBestRange){
                currBestRange = curr_range;
                bestIntersection = intersectionPoint;
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
  for (int i=0; i< num_ranges; i++) {

    float currLaserAngle = angle_min + i*angleIncrement;
    std::pair<Vector2f, float> cloudPointInfo = GetIntersectionPoint(LidarLoc, currLaserAngle, angle, range_min, range_max);
    scan[i] = cloudPointInfo.first;
     ranges[i] = cloudPointInfo.second;
  }
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
  for (int i = 0; i < num_ranges; i++){
      //std::cout << "RangeNum: " << i << " pred range " << predictedRanges[i] << " actual range " << ranges[i] << std::endl;
      float diff = predictedRanges[i] - ranges[i];
      float squaredDiff = std::pow(diff, 2);

      // Simple Observation Model
      currObservationLogProb += -squaredDiff/squared_laser_stddev_;

     /*
      // Robust Model here --- Switch to this if needed
      // NOTE: This might need more tuning  efforst becasue of 4 extra parameters
      if(diff < -s_min_ || diff > s_max_){
          currObservationLogProb += -std::numeric_limits<double>::infinity();
      }else if(diff < -d_short_){
          currObservationLogProb += d_short_log_prob_;
      }
      else if(diff > d_long_){
          currObservationLogProb += d_long_log_prob_;
      }
      else {
          currObservationLogProb += -squaredDiff/squared_laser_stddev_;
      }*/

  }
  
  // accounting for correlation in the joint probability
  p.weight += gamma_*currObservationLogProb;
}

void ParticleFilter::Resample(const std::vector<double>& normWeightProbs) {
  // Initialize the D for calculating range
  std::vector<double> particles_weight_new;
  particles_weight_new.reserve(particles_.size());   //creating an empty array for calculating Weight*D
  std::vector<Particle> new_particles = {};                 //initialising a vector for particles
  double prev_particle_range_upper_bound = 0;
  for (size_t i = 0; i < particles_.size(); i++) {
      particles_weight_new[i] = normWeightProbs[i] + prev_particle_range_upper_bound;
      prev_particle_range_upper_bound = particles_weight_new[i];
  }

  //this loop will run for the given number of particles for resampling with unweighing
  for (size_t i =0; i<particles_.size(); i++) {
      float x = rng_.UniformRandom(0, 1);               //uniform random number generator
      for (size_t j = 0; j < particles_.size(); j++) {
          if (x < particles_weight_new[j]) {
              new_particles.push_back(particles_[j]);
              break;
          }
      }
  }
  particles_ = new_particles;
  //initialize all particle weights to be 1/N
  for (size_t i = 0; i<particles_.size(); i++){
      particles_[i].weight = log(1/num_particles_);
  }
}


void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {

     /*
      * 1) Check if the car as traveled some distance
      * 2) Update if needed
      * 3) Check resampling
      * 4) Resample if needed  -- This resets the particle weights
      */
    
     // This should always be false and only set true when enters resampling
     justResampled_ = false;
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
            double bestWeightProb = std::exp(particles_[bestParticleIndex].weight);
            for (size_t i=0; i<= particles_.size(); i++) {
                double prob = std::exp(particles_[i].weight);
                normalizedProbWeights.push_back(prob/bestWeightProb);
            }

            //Resample
            Resample(normalizedProbWeights);
            justResampled_ = true;
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

      // Implementing odometry based motion model found on pg. 136 of Probabilistic Robotics book
      float prev_odom_x = prev_odom_loc_.x();
      float curr_odom_x = odom_loc.x();

      float prev_odom_y = prev_odom_loc_.y();
      float curr_odom_y = odom_loc.y();

      double delta_rot_1 = atan2(curr_odom_y - prev_odom_y, curr_odom_x - prev_odom_x) - prev_odom_angle_;
      double delta_trans = (odom_loc - prev_odom_loc_).norm();
      double delta_rot_2 = odom_angle - prev_odom_angle_ - delta_rot_1;

      // Keeping track of distance travelled since last update call
      dispFromLastUpdate_ += delta_trans;

      for (size_t i = 0; i < particles_.size(); i++) {

          // TODO ppts of this concept don't square delta_rot_1, delta_trans, or delta_rot_2. Should we?
          double delta_rot_1_std_dev =
                  sqrt((motion_model_alpha_1_ * pow(delta_rot_1, 2)) + (motion_model_alpha_2_ * pow(delta_trans, 2)));
          double delta_trans_std_dev = sqrt((motion_model_alpha_3_ * pow(delta_trans, 2)) +
                                            (motion_model_alpha_4_ * (pow(delta_rot_1, 2) + pow(delta_rot_2, 2))));
          double delta_rot_2_std_dev =
                  sqrt((motion_model_alpha_1_ * pow(delta_rot_2, 2)) + (motion_model_alpha_2_ * pow(delta_trans, 2)));

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
  // was received from the log. Initialg
  // ize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  map_.Load("maps/" + map_file + ".txt");
  odom_initialized_ = false;
  dispFromLastUpdate_ = 0;
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

    if (justResampled_) {
        loc = particles_[lastUpdateBestParticleIndex_].loc;
        angle = particles_[lastUpdateBestParticleIndex_].angle;
        return;
    }

    // Getting the particle with highest weight
    double bestWeight = -std::numeric_limits<double>::infinity();
    size_t bestParticleIndex = 0;
    for(size_t i =0; i<particles_.size();i++) {
        if(particles_[i].weight >= bestWeight) {
            bestWeight = particles_[i].weight;
            bestParticleIndex = i;
        }
    }
    loc = particles_[bestParticleIndex].loc;
    angle = particles_[bestParticleIndex].angle;
}
}  // namespace particle_filter
