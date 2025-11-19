#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <gridmap/gridmap.h>
#include <env_build/geom_env.h>
#include <memory>
using namespace pomp_planning_octomap;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_planning_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    readParem(nh);

    std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

    

    build_env_pointcloud(nh,cloud.get());

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}