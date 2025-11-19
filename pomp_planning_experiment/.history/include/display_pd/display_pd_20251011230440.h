#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
class PointsCloudPublisher
{
public:
    PointsCloudPublisher(ros::NodeHandle &nh,
                         const std::string &topic = "/points",
                         const std::string &frame_id = "map")
        : frame_id_(frame_id)
    {
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
    }

    // Accepts 3xN or Nx3 matrix of points.
    void loop_pd(const Eigen::Ref<const Eigen::MatrixX3d> &M)
    {

        int N = M.rows();

        cloud->points.reserve(N);

        for (int i = 0; i < N; ++i)
            cloud->points.emplace_back(static_cast<float>(M(i, 0)),
                                       static_cast<float>(M(i, 1)),
                                       static_cast<float>(M(i, 2)));

        // cloud.width = static_cast<uint32_t>(cloud.points.size());
        // cloud.height = 1;
        // cloud.is_dense = true;

        // sensor_msgs::PointCloud2 msg;
        // pcl::toROSMsg(cloud, msg);
        // msg.header.stamp = ros::Time::now();
        // msg.header.frame_id = frame_id_;
        // pub_.publish(msg);
    }

    void publish()
    {

        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        pub_.publish(msg);
    }

    void setFrameId(const std::string &frame_id) { frame_id_ = frame_id; }

private:
    ros::Publisher pub_;
    std::string frame_id_;
    std::unique_ptr<Cloud> cloud;
};