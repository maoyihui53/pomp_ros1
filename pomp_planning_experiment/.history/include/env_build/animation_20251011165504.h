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

    Eigen::MatrixX3d geom_pos;
    Eigen::Vector3d center;
    double half_size; // radius or half size l

    geom(int N, Eigen::Vector3d c, double half_size)
    {
        geom_pos.resize(N, 3);
    }
};

class geom_env
{
    int idx;
    std::vector<geom> cloud_boxes;

public:
    geom_env()
    {
        idx = 0;
        cloud_boxes.reserve(boxes_param.num_boxes);
    }
};

class animation_frames
{

private:
    std::vector<geom_env> frames;

public:
    animation_frames(int frame_num)
    {
        frames.reserve(frame_num);
    }
};