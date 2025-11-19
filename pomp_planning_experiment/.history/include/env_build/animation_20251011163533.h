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
    Eigen::Vector3d center;
    double half_size; // radius or half size l
};

class geom_env
{
    int idx;
    std::vector<geom> cloud_boxes;

    geom_env()
    {
        idx = 0;
        cloud_boxes.reserve(boxes_param.num_boxes);
    }


};

class animation_frames
{
    std::vector<geom_env> frames;

public:
    animation_frames(int frame_num, pcl::PointCloud<pcl::PointXYZ> *cloud)
    {
        frames.reserve(frame_num);
        
    }

    
};