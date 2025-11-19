#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <utils/util.h>
#include <Eigen/Dense>
#include <vector>

class geom_env
{

    Eigen::Matrix3d cloud_balls;
    Eigen::Matrix3d cloud_cones;
    Eigen::Matrix3d cloud_boxes;

    geom_env()
    {
    }
};
class animation_frames
{
    std::vector<geom_env> frames;
};