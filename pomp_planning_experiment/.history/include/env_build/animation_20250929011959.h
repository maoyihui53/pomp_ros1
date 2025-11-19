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

class geom
{

    Eigen::Matrix3d geom_pos;
};

class geom_env
{

    std::vector<geom> cloud_balls;
    std::vector<geom> cloud_cones;
    std::vector<geom> cloud_boxes;

    geom_env(int ball_num, int cones_num, int boxes_num)
    {
    }
};

class animation_frames
{
    std::vector<geom_env> frames;
};