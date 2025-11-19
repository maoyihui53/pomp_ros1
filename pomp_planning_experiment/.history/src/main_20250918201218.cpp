#include <ros/ros.h>
#include <param/paramReader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_mapping_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    read_param(nh);


    std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();
    return 0;
}