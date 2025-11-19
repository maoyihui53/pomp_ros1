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


    std::unique_ptr<geom_env> cubes_= std::make_unique<geom_env>(); 

    build_env_pointcloud(nh, cubes.get());

    // for (auto &p : cloud->points)
    // {
    //     // read
    //     float x = p.x, y = p.y, z = p.z;

    //     // modify
    //     p.x += 1.0f;
    // }

    std::cout << "point size "<< cloud->points.size()<<std::endl;

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}