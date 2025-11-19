#include <ros/ros.h>
#include <param/paramReader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_mapping_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    readParem(nh);

    int test=0;


    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;
    std::cout <<"test: " <<test_here<< std::endl;

    ros::spin();
    return 0;
}