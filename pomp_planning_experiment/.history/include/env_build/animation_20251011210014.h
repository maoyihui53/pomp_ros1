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

public:
    geom(int N)
    {
        geom_pos.resize(N, 3);
    }
};

class geom_env
{
    int idx;
    std::vector<geom> cloud_boxes;
    std::vector<Eigen::Vector3d> centers;
    std::vector<double> half_sizes;

public:
    geom_env()
    {
        idx = 0;
        cloud_boxes.reserve(boxes_param.num_boxes);
        centers.reserve(boxes_param.num_boxes);
        half_sizes(boxes_param.num_boxes);
    }

    void set_centers(const Eigen::Vector3d &center)
    {
        centers.emplace_back(centers);
    }

    void set_half_size(double hs)
    {
        half_sizes.emplace_back(hs);
    }

    void push_back(const geom &cube)
    {
        cloud_boxes.emplace_back(cube);
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