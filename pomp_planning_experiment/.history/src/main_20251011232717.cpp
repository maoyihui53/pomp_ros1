#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <gridmap/gridmap.h>
#include <env_build/geom_env.h>
#include <display_pd/display_pd.h>
#include <memory>

using namespace pomp_planning_octomap;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_planning_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    readParem(nh);

    std::unique_ptr<animation_frames> frames = std::make_unique<animation_frames>();

    build_env_pointcloud(frames);

    frames->print_info();
    std::unique_ptr<PointsCloudPublisher> pdPublisher = std::make_unique<PointsCloudPublisher>(nh, frames->pd_amount_in_frame(0));
    geom_env *current_frame = frames->get_frame_ptr(0);

    if (current_frame == nullptr)
        return -1;
    pdPublisher->read_frame(current_frame);

    pdPublisher->publish();

    // for (auto &p : cloud->points)
    // {
    //     // read
    //     float x = p.x, y = p.y, z = p.z;

    //     // modify
    //     p.x += 1.0f;
    // }

    // std::cout << "point size "<< cloud->points.size()<<std::endl;

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}