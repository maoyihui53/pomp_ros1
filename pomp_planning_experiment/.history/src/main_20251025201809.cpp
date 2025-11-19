#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <gridmap/gridmap.h>
#include <env_build/geom_env.h>
#include <display_pd/display_pd.h>
#include <jps_basis/data_utils.h>
#include <jps_tbb/jps_planner/jps_planner/jps_planner.h>
#include <jps_tbb/jps_planner/distance_map_planner/distance_map_planner.h>

#include <jps/jps_planner/jps_planner/jps_planner.h>
#include <jps/jps_planner/distance_map_planner/distance_map_planner.h>
#include <read_pcd/readPcd.h>

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

    frames->set_up();

    // std::cout << "resolution: " << resolution << std::endl;

    // frames->next_frame();

    // // frames->print_info();
    std::unique_ptr<PointsCloudPublisher> pdPublisher = std::make_unique<PointsCloudPublisher>(nh, frames->pd_amount_in_frame(0));

    // geom_env *current_frame = frames->get_frame_ptr(1);

    // if (current_frame == nullptr)
    //     return -1;
    // pdPublisher->read_frame(current_frame);

    // pdPublisher->publish();

    // for (auto &p : cloud->points)
    // {
    //     // read
    //     float x = p.x, y = p.y, z = p.z;

    //     // modify
    //     p.x += 1.0f;
    // }

    // std::cout << "point size "<< cloud->points.size()<<std::endl;

    // std::cout<<"num_balls: "<<balls_param.num_balls<<std::endl;

    ros::Rate rate(10); // 1 Hz

    int frame_idx = 0;
    while (ros::ok())
    {
        Eigen::Matrix3Xd pd = frames->get_pd_in_frame(0);
        // std::cout << "pd size:" << pd.rows() << std::endl;

        std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(pd);

        env_info *env = reader->getEnvInfo(resolution);

        // env->print_info();
        std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
        pomp_octomap->insertPcd(env->points_array);
        pomp_octomap->mapping();

        std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
        grid_map->insertPcd(env->points_array);

        Eigen::Array3d min_corner = grid_map->getMinCorner();
        Eigen::Array3i map_dim = grid_map->getMapDim();

        Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
        Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
        decimal_t resolution = grid_map->getResolution();

        std::shared_ptr<JPS::VoxelMapUtil> jps_map_util = std::make_shared<JPS::VoxelMapUtil>();
        jps_map_util->setMap(origin, dim, grid_map->getGridMap(), resolution);
        std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
        planner_ptr->setMapUtil(jps_map_util);

        // Set collision checking function
        planner_ptr->updateMap();
        // auto start_plan_time = std::chrono::high_resolution_clock::now();
        // valid_grid_jps_path = planner_ptr->plan(start, goal, 1, true);
        // auto end_plan_time = std::chrono::high_resolution_clock::now();
        // duration_grid_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

        geom_env *current_frame = frames->get_frame_ptr(frame_idx);
        if (current_frame == nullptr)
            return -1;
        pdPublisher->read_frame(current_frame);
        pdPublisher->publish();

        frames->next_frame();

        frame_idx++;
        rate.sleep();
        ros::spinOnce();
    }

    // ros::spin();

    return 0;
}