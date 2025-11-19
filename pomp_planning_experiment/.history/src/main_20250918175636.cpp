#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");

    ros::spin();
    return 0;
}