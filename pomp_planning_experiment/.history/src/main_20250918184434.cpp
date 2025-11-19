#include <ros/ros.h>
#include <param/paramReader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_mapping_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");

    ros::spin();
    return 0;
}