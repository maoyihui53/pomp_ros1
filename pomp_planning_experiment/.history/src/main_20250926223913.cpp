#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <gridmap/gridmap.h>
#include <read_pcd/readPcd.h>

using namespace pomp_planning_octomap;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pomp_mapping_experiments_geometry");
    ros::NodeHandle nh;

    ROS_INFO("Node has started.");
    readParem(nh);

    std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(nh);

    env_info *env = reader->getEnvInfo(resolution);
    env->print_info();
    std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
    pomp_octomap->insertPcd(env->points_array);

    std::cout << "pomp octomap mapping free amount percent:" << pomp_octomap->get_mapping_free_percent() << std::endl;

    std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
    grid_map->insertPcd(env->points_array);

    std::cout << "grid map mapping free amount percent:" << grid_map->get_mapping_free_percent() << std::endl;

    reader->publishCloud();

    // build_env_pointcloud(nh);

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}