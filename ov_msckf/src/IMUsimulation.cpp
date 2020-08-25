/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */



#include <csignal>

#include "sim/Simulator.h"
#include "core/VioManager.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"

#include <message/UVmsg.h>
#include <message/UVsmsg.h>
#include <message/UVListstamped.h>
#ifdef ROS_AVAILABLE
#include <ros/ros.h>
#include "core/RosVisualizer.h"
#include "utils/parse_ros.h"
#include <chrono>
#include <thread>
#endif


using namespace ov_msckf;


Simulator* sim;
VioManager* sys;
#ifdef ROS_AVAILABLE
RosVisualizer* viz;
#endif

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}


// Main function
int main(int argc, char** argv)
{
    // Read in our parameters
    VioManagerOptions params;
#ifdef ROS_AVAILABLE
    ros::init(argc, argv, "imu_simulation");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);
#else
    params = parse_command_line_arguments(argc, argv);
#endif

    // Create our VIO system
    sim = new Simulator(params);
    sys = new VioManager(params);
#ifdef ROS_AVAILABLE
    viz = new RosVisualizer(nh, sys, sim);
    ros::Publisher pub_measfeat;
    pub_measfeat = nh.advertise<message::UVListstamped>("/message/measfeat", 2);
    ROS_INFO("Publishing: %s", pub_measfeat.getTopic().c_str());
#endif

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Get initial state
    Eigen::Matrix<double, 17, 1> imustate;
    bool success = sim->get_state(sim->current_timestamp(),imustate);
    if(!success) {
        printf(RED "[SIM]: Could not initialize the filter to the first state\n" RESET);
        printf(RED "[SIM]: Did the simulator load properly???\n" RESET);
        std::exit(EXIT_FAILURE);
    }

    // Since the state time is in the camera frame of reference
    // Subtract out the imu to camera time offset
    imustate(0,0) -= sim->get_true_paramters().calib_camimu_dt;

    // Initialize our filter with the groundtruth
    sys->initialize_with_gt(imustate);

    //===================================================================================
    //===================================================================================
    //===================================================================================
    std::string path_imu_meas;
    std::ofstream of_imu_meas;

    nh.param<std::string>("path_imu_meas", path_imu_meas, "imumeas.csv");
    if (boost::filesystem::exists(path_imu_meas))
        boost::filesystem::remove(path_imu_meas);

    // open stream for writing out imu data
    of_imu_meas.open(path_imu_meas.c_str());
    printf("Writing estimated IMU pose to: %s\n", path_imu_meas.c_str());

    // Buffer our camera image
    double buffer_timecam = -1;
    std::vector<int> buffer_camids;
    std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> buffer_feats;
    unsigned int seq_featList = 0;
    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);
#ifdef ROS_AVAILABLE
    while(sim->ok() && ros::ok()) {
#else
    while(sim->ok()) {
#endif

        // ROS_INFO("Sleeping for 1ms for visualization");
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim->get_next_imu(time_imu, wm, am);
        if(hasimu) {
            sys->feed_measurement_imu(time_imu, wm, am);
            // printf("Writing IMU data...\n");
            of_imu_meas << std::setprecision(14) << time_imu << "," << am(0) << "," << am(1) << ","
                << am(2) << "," << wm(0) << "," << wm(1) << "," << wm(2) << "\n";

#ifdef ROS_AVAILABLE
            viz->visualize_odometry(time_imu);
#endif
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim->get_next_cam(time_cam, camids, feats);

        if(hascam) {
            if(buffer_timecam != -1) {
                sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
#ifdef ROS_AVAILABLE
                viz->visualize_imu(wm, am);
                std::string path_feats;
                path_feats = viz->path_featdir + "/" + std::to_string(seq_featList) + ".csv";
                if (boost::filesystem::exists(path_feats))
                    boost::filesystem::remove(path_feats);
                std::ofstream of_feats;
                of_feats.open(path_feats.c_str());

                message::UVListstamped uvsmsg;
                std::vector<message::UVsmsg> points;
                uvsmsg.header.stamp = ros::Time(time_cam);
                uvsmsg.header.seq = seq_featList;
                uvsmsg.header.frame_id = "cam";
                uvsmsg.cam_id = camids;

                // flatten features in different camera;
                std::vector<std::pair<size_t,Eigen::VectorXf>> featsInAllC;

                for (auto featsInC = feats.begin(); featsInC != feats.end(); ++featsInC) {
                    of_feats << uvsmsg.header.stamp << "," << uvsmsg.header.seq  << "\n";
                    std::vector<message::UVmsg> pointL;
                    message::UVsmsg pList;
                    for (auto it = featsInC->begin(); it != featsInC->end(); ++it) {
                        message::UVmsg point;
                        point.id = it->first;
                        point.u = it->second(0);
                        point.v = it->second(1);
                        pointL.push_back(point);
                        of_feats << it->first << "," << it->second(0) << "," << it->second(1) << "\n";
                        featsInAllC.push_back(*it);
                    }
                    pList.points = pointL;
                    points.push_back(pList);
                }
                uvsmsg.points = points;
                pub_measfeat.publish(uvsmsg);

                viz->publish_feats_inC(featsInAllC);

                ++seq_featList;
#endif
            }
            buffer_timecam = time_cam;
            buffer_camids = camids;
            buffer_feats = feats;
        }

    }


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // close stream for writing out imu measurement
    of_imu_meas.close();

    // Final visualization
#ifdef ROS_AVAILABLE
    viz->visualize_final();
    viz->save_featsgt();
    delete viz;
#endif

    // Finally delete our system
    delete sim;
    delete sys;


    // Done!
    return EXIT_SUCCESS;

}

