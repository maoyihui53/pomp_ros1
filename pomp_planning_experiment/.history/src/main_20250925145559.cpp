#include <ros/ros.h>
#include <param/paramReader.h>

#include <read_pcd/readPcd.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_mapping_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    readParem(nh);

    std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(nh);

    reader->publishCloud();

    // build_env_pointcloud(nh);

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}