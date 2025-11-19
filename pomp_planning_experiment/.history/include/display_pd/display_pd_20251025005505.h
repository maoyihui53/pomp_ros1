#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
#include <env_build/animation.h>
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
class PointsCloudPublisher
{
public:
    PointsCloudPublisher(ros::NodeHandle &nh, int pd_num = 150000,
                         const std::string &topic = "/points",
                         const std::string &frame_id = "map")
        : frame_id_(frame_id), cloud(std::make_unique<Cloud>())
    {

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
        cloud->points.reserve(pd_num);
    }

    void read_frame(geom_env *current_frame)
    {
        cloud->points.clear();
        for (auto &b : current_frame->get_boxes())
        {
            read_pd(b.get_geom_pos());
        }
    }

    void read_pd(const Eigen::Ref<const Eigen::Matrix3Xd> &M)
    {

        std::cout << "M rows:" << M.rows()<< "M cols:" << M.cols()<< std::endl;
        
        int N = M.rows();


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

    // void setFrameId(const std::string &frame_id) { frame_id_ = frame_id; }

private:
    ros::Publisher pub_;
    std::string frame_id_;
    std::unique_ptr<Cloud> cloud;
};