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

    double pomp_percent_sum = 0.0;
    double grid_percent_sum = 0.0;

    double pomp_percent_array[200];
    double grid_percent_array[200];

    for (int i = 0; i < 1; i++)
    {

        std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(nh, i);

        env_info *env = reader->getEnvInfo(resolution);
        env->print_info();
        std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
        pomp_octomap->insertPcd(env->points_array);
        pomp_octomap->mapping();

        // std::cout << "pomp octomap mapping free amount percent:" << pomp_octomap->get_mapping_free_percent() << std::endl;
        pomp_percent_array[i] = pomp_octomap->get_mapping_free_percent();
        pomp_percent_sum = pomp_percent_sum + pomp_octomap->get_mapping_free_percent();

        std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
        grid_map->insertPcd(env->points_array);
        grid_percent_array[i] = grid_map->get_mapping_free_percent();
        grid_percent_sum = grid_percent_sum + grid_map->get_mapping_free_percent();

        // std::cout << "grid map mapping free amount percent:" << grid_map->get_mapping_free_percent() << std::endl;
    }

    // double pomp_percent = pomp_percent_sum / 200.0;
    // double grid_percent = grid_percent_sum / 200.0;

    // double pomp_std = 0.0;
    // double grid_std = 0.0;
    // for (size_t i = 0; i < 200; i++)
    // {
    //     pomp_std += (pomp_percent_array[i] - pomp_percent) * (pomp_percent_array[i] - pomp_percent);
    //     grid_std += (grid_percent_array[i] - grid_percent) * (grid_percent_array[i] - grid_percent);
    // }
    // pomp_std = sqrt(pomp_std / 199.0);
    // grid_std = sqrt(grid_std /199.0);



    // std::cout << "pomp octomap average mapping free amount percent:" << pomp_percent<< std::endl;
    // std::cout << "pomp octomap std:" << pomp_std << std::endl;
    // std::cout << "grid map average mapping free amount percent:" << grid_percent << std::endl;
    // std::cout << "grid map std:" << grid_std << std::endl;

    


    reader->publishCloud();

    // build_env_pointcloud(nh);

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::spin();

    return 0;
}