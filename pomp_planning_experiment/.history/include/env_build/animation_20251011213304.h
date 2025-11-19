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
    int idx;

public:
    geom(int N)
    {
        geom_pos.resize(N, 3);
        idx = 0;
    }

    void push_back(const Eigen::Vector3d &pos)
    {
        geom_pos.row(idx) = pos.transpose(); //
        idx++;
    }

    Eigen::MatrixX3d get_geom_pos()
    {
        return geom_pos;
    }

    int amount()
    {
        return idx;
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
        half_sizes.reserve(boxes_param.num_boxes);
    }

    void set_centers(const Eigen::Vector3d &center)
    {
        centers.emplace_back(center);
    }

    void set_half_size(double hs)
    {
        half_sizes.emplace_back(hs);
    }

    void push_back(const geom &cube)
    {
        cloud_boxes.emplace_back(cube);
    }

    void print_info()
    {
        std::cout << "boxes amount: " << cloud_boxes.size() << std::endl;
        std::cout << "center amount: " << centers.size() << std::endl;
        std::cout << "half_size: " << half_sizes.size() << std::endl;
        int idx = 0;
        for (auto &b : cloud_boxes)
        {
            idx = idx + b.amount();
        }
        std::cout << "points amount: " << idx << std::endl;
    }
};

class animation_frames
{

private:
    std::vector<geom_env> frames;

public:
    animation_frames(int frame_num = 1000)
    {
        frames.reserve(frame_num);
        std::unique_ptr<geom_env> cubes_ = std::make_unique<geom_env>();

        build_env_pointcloud(cubes_);
        frames.emplace_back(cubes_);
        frames[0].print_info();
    }
};