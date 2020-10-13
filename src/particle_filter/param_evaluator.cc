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
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <boost/thread.hpp>

#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

#include <iostream>
#include <iomanip>
#include <ctime>

#include <std_msgs/Empty.h>

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;;
using visualization::DrawLine;

bool run_ = true;
amrl_msgs::Localization2DMsg reference_;
Eigen::Vector2f reference_loc_;
amrl_msgs::Localization2DMsg actual_;
ros::NodeHandle *n_;
ros::Publisher visualization_publisher_;

VisualizationMsg vis_msg_;
ros::Subscriber reference_sub_;
ros::Subscriber loc_sub_;
ros::Subscriber set_pose_sub_;
ros::Subscriber set_nav_target_sub_;
ros::Publisher reset_params_pub_;
double total_error_;
double total_measurements_;

bool initialized_;
bool new_ref_;


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
/**
 *ROS parameter name for d_long in observation model
 */
const std::string kObsDlongParamName = "observation_d_long";
/**
 * ROS parameter name for D - distance between 2 update calls
 */
const std::string kObsDParamName = "observation_d";
/**
 * ROS Parameter name for K - number of updates between two resample calls
 */
const std::string kObsKParamName = "observation_k";

/**
 * ROS parameter name for the distance that a particle can be (at most) from the maximum weight particle to be
 * included in the weighted average used to get the localization estimate.
 */
const std::string kGetLocationAveragingDistParamName = "get_loc_averaging_dist";

struct ParamSetting {
    int num_particles_;
    double initial_x_stddev_;
    double initial_y_stddev_;
    double initial_theta_stddev_;
    double gamma_;
    float squared_laser_stddev_;
    float d_short_;
    float d_long_;
    float obs_d_;
    int obs_k_;
    double motion_model_alpha_1_;
    double motion_model_alpha_2_;
    double motion_model_alpha_3_;
    double motion_model_alpha_4_;
    double get_loc_averaging_dist_;
};

void publishReferencePos(const amrl_msgs::Localization2DMsg &msg) {
    double len = 0.3;
    Vector2f p0(msg.pose.x, msg.pose.y);
    Vector2f p1(msg.pose.x + len*cos(msg.pose.theta), msg.pose.y + len*sin(msg.pose.theta));
    visualization::DrawLine(p0, p1, 0xa503fc, vis_msg_);
    visualization_publisher_.publish(vis_msg_);
}

void SetNavTargetCallback(const amrl_msgs::Localization2DMsg &msg) {
    new_ref_ = true;
    reference_ = msg;
    reference_loc_ = Eigen::Vector2f(msg.pose.x, msg.pose.y);
    publishReferencePos(msg);
}

void ReferenceCallback(const amrl_msgs::Localization2DMsg& msg) {
    new_ref_ = true;
    reference_ = msg;
    reference_loc_ = Eigen::Vector2f(msg.pose.x, msg.pose.y);
    publishReferencePos(msg);
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
    Eigen::Vector2f loc = Eigen::Vector2f(msg.pose.x, msg.pose.y);
    double error = (loc - reference_loc_).norm();
    if (!isnan(error)) {
        if (new_ref_) {
            new_ref_ = false;
            total_error_ += error;
            total_measurements_++;
        }
    }
}

void SetPoseCallback(const amrl_msgs::Localization2DMsg &msg) {
    visualization::ClearVisualizationMsg(vis_msg_);
    total_error_ = 0;
    total_measurements_ = 0;
    initialized_ = true;
    ROS_INFO_STREAM("Initialized");
}

void playRosbag(const std::string &rosbag_name) {
    initialized_ = false;
    new_ref_ = false;
    std::string run_cmd = "rosbag play " + rosbag_name + " --topics /initialpose /odom /scan /set_nav_target /set_pose";
    ROS_INFO_STREAM("System result: " << system(run_cmd.c_str()));
}

double getAbsoluteTrajectoryError(const std::string &rosbag_name) {
    total_error_ = 0;
    total_measurements_ = 0;

    boost::thread rosbag_thread(playRosbag, rosbag_name);
    rosbag_thread.join();
    return total_error_ / total_measurements_;
}


void SignalHandler(int) {
    if (!run_) {
        printf("Force Exit.\n");
        exit(0);
    }
    printf("Exiting.\n");
    run_ = false;
}

void setParams(const ParamSetting &params) {
    n_->setParam(kNumParticlesParamName, params.num_particles_);
    n_->setParam(kInitialXStdDevParamName, params.initial_x_stddev_);
    n_->setParam(kInitialYStdDevParamName, params.initial_y_stddev_);
    n_->setParam(kInitialThetaStdDevParamName, params.initial_theta_stddev_);
    n_->setParam(kObsGammaParamName, params.gamma_);
    n_->setParam(kObsStdDevSquaredParamName, params.squared_laser_stddev_);
    n_->setParam(kObsDshortParamName, params.d_short_);
    n_->setParam(kObsDlongParamName, params.d_long_);
    n_->setParam(kObsDParamName, params.obs_d_);
    n_->setParam(kObsKParamName, params.obs_k_);
    n_->setParam(kMotionModelAlpha1ParamName, params.motion_model_alpha_1_);
    n_->setParam(kMotionModelAlpha2ParamName, params.motion_model_alpha_2_);
    n_->setParam(kMotionModelAlpha3ParamName, params.motion_model_alpha_3_);
    n_->setParam(kMotionModelAlpha4ParamName, params.motion_model_alpha_4_);
    n_->setParam(kGetLocationAveragingDistParamName, params.get_loc_averaging_dist_);
}

void writeCsvHeader(const std::string &file_name) {
    std::ofstream csv_file(file_name, std::ios::app);
    csv_file << "Ros bag name" << ", " << kNumParticlesParamName << ", " << kInitialXStdDevParamName << ", "
             << kInitialYStdDevParamName << ", " << kInitialThetaStdDevParamName << ", "  << kObsGammaParamName << ", "
             << kObsStdDevSquaredParamName << ", "  << kObsDshortParamName << ", "  << kObsDlongParamName << ", "
             << kObsDParamName << ", "  << kObsKParamName << ", "  << kMotionModelAlpha1ParamName << ", "
             << kMotionModelAlpha2ParamName<< ", "  << kMotionModelAlpha3ParamName << ", "  << kMotionModelAlpha4ParamName
             << ", "  << kGetLocationAveragingDistParamName << ", " << "Average Trajectory Error" << "\n";
    csv_file.close();
}

void outputToCsv(const std::string &file_name, const std::string &rosbag_name, const ParamSetting &params, const double &ate) {
    std::ofstream csv_file(file_name, std::ios::app);
    csv_file << rosbag_name << ", " << params.num_particles_ << ", " << params.initial_x_stddev_ << ", "
             << params.initial_y_stddev_ << ", " << params.initial_theta_stddev_ << ", "  << params.gamma_ << ", "
             << params.squared_laser_stddev_ << ", "  << params.d_short_ << ", "  << params.d_long_ << ", "
             << params.obs_d_ << ", "  << params.obs_k_ << ", "  << params.motion_model_alpha_1_ << ", "
             << params.motion_model_alpha_2_ << ", "  << params.motion_model_alpha_3_ << ", "  << params.motion_model_alpha_4_
             << ", "  << params.get_loc_averaging_dist_ << ", " << ate << "\n";
    csv_file.close();
}

void setParametersAndRunRosbags(const std::string &csv_file, const std::vector<std::string> &rosbag_names, const ParamSetting &params) {
    setParams(params);
    reset_params_pub_.publish(std_msgs::Empty());
    ros::Duration(1).sleep();
    std::vector<double> all_errors;
    double error_avg = 0;

    for (const std::string &bag_name : rosbag_names) {
        double error = getAbsoluteTrajectoryError(bag_name);
        error_avg += error;
        all_errors.emplace_back(error);
        outputToCsv(csv_file, bag_name, params, error);
    }
    error_avg /= all_errors.size();
    outputToCsv(csv_file, "all", params, error_avg);
}

void runOnManyParamSets(const std::string &csv_file, const std::vector<std::string> &rosbag_names) {

    // NOTE: Put parameter values that should be tested out here (if you have many, you'll need to reorder the for loops to prioritize the variables you want to vary most.
    std::vector<int> particle_nums = {75};
    std::vector<double> initial_x_stddevs = {1.0};
    std::vector<double> initial_y_stddevs = {1.0};
    std::vector<double> initial_theta_stddevs = {0.4};
    std::vector<double> gammas = {0.001, 0.005};
    std::vector<float> squared_laser_stddevs = {0.1};
    std::vector<float> d_shorts = {1.0, 2.0, 0.5};
    std::vector<float> d_longs = {2.0};
    std::vector<float> obs_ds = {0.05, 0.1, 0.2};
    std::vector<int> obs_ks = {3, 10, 1, 15, 20};
    std::vector<double> motion_model_alpha_1s = {0.2, 0.1};
    std::vector<double> motion_model_alpha_2s = {0.2, 0.1};
    std::vector<double> motion_model_alpha_3s = {0.06, 0.1, 0.06};
    std::vector<double> motion_model_alpha_4s = {0.00005, 0.01, 0.02};
    std::vector<double> get_loc_averaging_dists = {0.5, 0.25, 1.0};
    ParamSetting params;
    for (const auto &particle_num : particle_nums) {
        ROS_INFO_STREAM("Num particles " << particle_num);
        for (const auto &get_loc_averaging_dist : get_loc_averaging_dists) {
            for (const auto &initial_x_stddev : initial_x_stddevs) {
                for (const auto &initial_y_stddev : initial_y_stddevs) {
                    for (const auto &initial_theta_stddev : initial_theta_stddevs) {
                        for (const auto &squared_laser_stddev : squared_laser_stddevs) {
                            ROS_INFO_STREAM("Laser variance " << squared_laser_stddev);
                            for (const auto &d_long : d_longs) {
                                ROS_INFO_STREAM("D long " << d_long);
                                for (const auto &d_short : d_shorts) {
                                    ROS_INFO_STREAM("D short " << d_short);
                                    for (const auto &obs_k : obs_ks) {
                                        ROS_INFO_STREAM("obs k " << obs_k);
                                        for (const auto &obs_d : obs_ds) {
                                            ROS_INFO_STREAM("obs d " << obs_d);
                                            for (const auto &motion_model_alpha_1 : motion_model_alpha_1s) {
                                                ROS_INFO_STREAM("Motion model 1 " << motion_model_alpha_1);
                                                for (const auto &motion_model_alpha_2 : motion_model_alpha_2s) {
                                                    ROS_INFO_STREAM("Motion model 2 " << motion_model_alpha_2);
                                                    for (const auto &motion_model_alpha_3 : motion_model_alpha_3s) {
                                                        ROS_INFO_STREAM("Motion model 3 " << motion_model_alpha_3);
                                                        for (const auto &motion_model_alpha_4 : motion_model_alpha_4s) {
                                                            ROS_INFO_STREAM("Motion model 4 " << motion_model_alpha_4);
                                                            for (const auto &gamma : gammas) {
                                                                ROS_INFO_STREAM("Gamma  " << gamma);
                                                                params.num_particles_ = particle_num;
                                                                params.initial_x_stddev_ = initial_x_stddev;
                                                                params.initial_y_stddev_ = initial_y_stddev;
                                                                params.initial_theta_stddev_ = initial_theta_stddev;
                                                                params.gamma_ = gamma;
                                                                params.squared_laser_stddev_ = squared_laser_stddev;
                                                                params.d_short_ = d_short;
                                                                params.d_long_ = d_long;
                                                                params.obs_d_ = obs_d;
                                                                params.obs_k_ = obs_k;
                                                                params.motion_model_alpha_1_ = motion_model_alpha_1;
                                                                params.motion_model_alpha_2_ = motion_model_alpha_2;
                                                                params.motion_model_alpha_3_ = motion_model_alpha_3;
                                                                params.motion_model_alpha_4_ = motion_model_alpha_4;
                                                                params.get_loc_averaging_dist_ = get_loc_averaging_dist;
                                                                ROS_INFO_STREAM("New param set");

                                                                for (int i = 0; i < 3; i++) {
                                                                    setParametersAndRunRosbags(csv_file, rosbag_names,
                                                                                               params);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}



int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);
    // Initialize ROS.
    ros::init(argc, argv, "measure_error", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    n_ = &n;
    // NOTE: List rosbags you'd like to run on here
    std::vector<std::string> rosbags = {
            "~/Downloads/2020-10-09-16-49-39.bag",
            "~/Downloads/GDC1/2020-04-01-17-08-55.bag",
            "~/Downloads/GDC1/2020-04-01-17-09-24.bag",
            "~/Downloads/GDC1/2020-04-01-17-14-01.bag",
            "~/Downloads/GDC1/2020-04-01-17-14-23.bag",
            "~/Downloads/GDC1/2020-04-01-17-20-04.bag"
    };

    vis_msg_ = visualization::NewVisualizationMessage("map", "reference_loc");

    reference_sub_ = n_->subscribe(
            "/reference_localization",
            1,
            ReferenceCallback);
    loc_sub_ = n_->subscribe(
            "/localization",
            1,
            LocalizationCallback);

    set_pose_sub_ = n_->subscribe(
            "/set_pose",
            1,
            SetPoseCallback);

    set_nav_target_sub_ = n_->subscribe("/set_nav_target", 1, SetNavTargetCallback);

    visualization_publisher_ =
            n.advertise<VisualizationMsg>("visualization", 1);

    reset_params_pub_ = n.advertise<std_msgs::Empty>("/reload_params", 1);

    while (reset_params_pub_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
    std::string time_str = oss.str();
    std::string csv_file_name = "param_eval_" + time_str + ".csv";
    writeCsvHeader(csv_file_name);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    runOnManyParamSets(csv_file_name, rosbags);
    return 0;
}
