#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <env_build/geom_env.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <utils/util.h>

using namespace pomp_env;

class readMapPd
{

private:
    std::unique_ptr<env_info> env;

public:
    readMapPd(const Eigen::MatrixX3d &pd)
        : env(new env_info())
    {
        update_pd(pd);
    }

    void update_pd(const Eigen::MatrixX3d &pd)
    {
        // env->points_array = pd;
        // env->min_corner = env->points_array.rowwise().minCoeff();
        // env->max_corner = env->points_array.rowwise().maxCoeff();
        // env->center = (env->min_corner + env->max_corner) / 2.0;
        // env->adjust_to_center();
    }

    env_info *getEnvInfo(double resolution = 0.1)
    {
        env->computeMinDepth(resolution); // Example resolution, adjust as needed
        return env.get();
    }
};