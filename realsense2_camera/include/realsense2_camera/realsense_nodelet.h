// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <realsense2_camera/constants.h>
#include <realsense2_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <realsense2_camera/IMUInfo.h>
#include <realsense2_camera/realsense_node.h>
#include <csignal>
#include <eigen3/Eigen/Geometry>
#include <fstream>

namespace realsense2_camera
{
    inline void signalHandler(int signum)
    {
        ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating RealSense Node...");
        ros::shutdown();
        exit(signum);
    }

    class RealSenseNodelet : public nodelet::Nodelet
    {
    public:
        RealSenseNodelet();
        virtual ~RealSenseNodelet() {}

    private:
        virtual void onInit() override;
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        std::unique_ptr<RealSenseNode> _realSenseNode;
    };
}//end namespace
