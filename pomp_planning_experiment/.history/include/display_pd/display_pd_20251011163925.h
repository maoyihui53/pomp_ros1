#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class PointsCloudPublisher {
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  PointsCloudPublisher(ros::NodeHandle& nh,
                           const std::string& topic = "/points",
                           const std::string& frame_id = "map")
  : frame_id_(frame_id)
  {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
  }

  // Accepts 3xN or Nx3 matrix of points.
  void publish(const Eigen::Ref<const Eigen::MatrixXd>& M) {
    Cloud cloud;
    const bool cols_are_points = (M.rows() == 3);
    const bool rows_are_points = (M.cols() == 3);

    if (!cols_are_points && !rows_are_points) {
      ROS_ERROR_STREAM("[EigenPointsRvizPublisher] Expected 3xN or Nx3 matrix, got "
                       << M.rows() << "x" << M.cols());
      return;
    }

    const std::size_t N = cols_are_points ? static_cast<std::size_t>(M.cols())
                                          : static_cast<std::size_t>(M.rows());
    cloud.points.reserve(N);

    if (cols_are_points) {
      for (int j = 0; j < M.cols(); ++j)
        cloud.points.emplace_back(static_cast<float>(M(0,j)),
                                  static_cast<float>(M(1,j)),
                                  static_cast<float>(M(2,j)));
    } else { // rows_are_points
      for (int i = 0; i < M.rows(); ++i)
        cloud.points.emplace_back(static_cast<float>(M(i,0)),
                                  static_cast<float>(M(i,1)),
                                  static_cast<float>(M(i,2)));
    }

    cloud.width  = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    pub_.publish(msg);
  }

  void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }

private:
  ros::Publisher pub_;
  std::string frame_id_;
};