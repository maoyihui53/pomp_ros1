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
// #include <chrone>

#include <memory>

float get_distance(const vec_Vec3f &path)
{

    float total_dist = 0;
    for (size_t i = 1; i < path.size(); i++)
    {
        Vec3f current_node = path[i];
        Vec3f previous_node = path[i - 1];

        float dist = (current_node - previous_node).norm();
        total_dist = total_dist + dist;
        //  (current_node[0]-previous_node[0])*(current_node[0]-previous_node[0])
        //  +(current_node[1]-previous_node[1])*(current_node[1]-previous_node[1])
        //  +(current_node[2]-previous_node[2])*(current_node[2]-previous_node[2])
    }

    return total_dist;
}

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
    Vec3f start = {-20.0f, -20.0f, -20.f};
    Vec3f goal = {20.0f, 20.0f, 20.f};

    int frame_idx = 0;

    int grid_success = 0;
    int pomp_success_95 = 0;
    int pomp_success_75 = 0;
    int pomp_success_50 = 0;
    int pomp_success_25 = 0;

    // float grid_path_length = 0;
    // float pomp_path_length = 0;

    // bool grid_find = false;
    // bool pomp_find_95 = false;

    // float current_gird_path_dist = 0;
    // float current_pomp_path_dist = 0;

    // int both_find = 0;

    while (true)
    {
        Eigen::Matrix3Xd pd = frames->get_pd_in_frame(frame_idx);
        // std::cout << "pd size:" << pd.rows() << std::endl;

        std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(pd);

        env_info *env = reader->getEnvInfo(resolution);
        {
            // env->print_info();
            std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
            pomp_octomap->insertPcd(env->points_array, 0.95);
            pomp_octomap->mapping();

            vec_Vec3f path_grid;
            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            // auto start_plan_time = std::chrono::high_resolution_clock::now();
            bool valid_pomp_jps_path = planner_ptr->plan(start, goal, 1, true);
            // auto end_plan_time = std::chrono::high_resolution_clock::now();
            // duration_pomp_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_pomp_jps_path)
            {
                path_grid = planner_ptr->getPath();

                // current_pomp_path_dist = get_distance(path_grid);

                // pomp_find = true;
                pomp_success_95++;

                // std::cout << "Grid JPS found path with " << path_grid.size() << " waypoints." << std::endl;
                // for (int i = 0; i < path_grid.size(); i++)
                // {
                //     std::cout << "Waypoint " << i << ": (" << path_grid[i][0] << ", " << path_grid[i][1] << ", " << path_grid[i][2] << ")" << std::endl;
                // }

                std::cout << "Pomp JPS 95 planning successful." << std::endl;
            }
            else
            {
                // pomp_find = false;
                std::cout << "Pomp JPS did not find a valid path." << std::endl;
            }
        }


        // 75

        {
            // env->print_info();
            std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
            pomp_octomap->insertPcd(env->points_array, 0.75);
            pomp_octomap->mapping();

            vec_Vec3f path_grid;
            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            // auto start_plan_time = std::chrono::high_resolution_clock::now();
            bool valid_pomp_jps_path = planner_ptr->plan(start, goal, 1, true);
            // auto end_plan_time = std::chrono::high_resolution_clock::now();
            // duration_pomp_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_pomp_jps_path)
            {
                path_grid = planner_ptr->getPath();

                // current_pomp_path_dist = get_distance(path_grid);

                // pomp_find = true;
                pomp_success_75++;

                // std::cout << "Grid JPS found path with " << path_grid.size() << " waypoints." << std::endl;
                // for (int i = 0; i < path_grid.size(); i++)
                // {
                //     std::cout << "Waypoint " << i << ": (" << path_grid[i][0] << ", " << path_grid[i][1] << ", " << path_grid[i][2] << ")" << std::endl;
                // }

                std::cout << "Pomp 75 JPS planning successful." << std::endl;
            }
            else
            {
                // pomp_find = false;
                std::cout << "Pomp JPS did not find a valid path." << std::endl;
            }
        }


        // 50

        {
            // env->print_info();
            std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
            pomp_octomap->insertPcd(env->points_array, 0.5);
            pomp_octomap->mapping();

            vec_Vec3f path_grid;
            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            // auto start_plan_time = std::chrono::high_resolution_clock::now();
            bool valid_pomp_jps_path = planner_ptr->plan(start, goal, 1, true);
            // auto end_plan_time = std::chrono::high_resolution_clock::now();
            // duration_pomp_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_pomp_jps_path)
            {
                path_grid = planner_ptr->getPath();

                // current_pomp_path_dist = get_distance(path_grid);

                // pomp_find = true;
                pomp_success_50++;

                // std::cout << "Grid JPS found path with " << path_grid.size() << " waypoints." << std::endl;
                // for (int i = 0; i < path_grid.size(); i++)
                // {
                //     std::cout << "Waypoint " << i << ": (" << path_grid[i][0] << ", " << path_grid[i][1] << ", " << path_grid[i][2] << ")" << std::endl;
                // }

                std::cout << "Pomp 50 JPS planning successful." << std::endl;
            }
            else
            {
                // pomp_find = false;
                std::cout << "Pomp JPS did not find a valid path." << std::endl;
            }
        }

        // 25

            

        {
            // env->print_info();
            std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
            pomp_octomap->insertPcd(env->points_array, 0.25);
            pomp_octomap->mapping();

            vec_Vec3f path_grid;
            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            // auto start_plan_time = std::chrono::high_resolution_clock::now();
            bool valid_pomp_jps_path = planner_ptr->plan(start, goal, 1, true);
            // auto end_plan_time = std::chrono::high_resolution_clock::now();
            // duration_pomp_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_pomp_jps_path)
            {
                path_grid = planner_ptr->getPath();

                // current_pomp_path_dist = get_distance(path_grid);

                // pomp_find = true;
                pomp_success_25++;

                // std::cout << "Grid JPS found path with " << path_grid.size() << " waypoints." << std::endl;
                // for (int i = 0; i < path_grid.size(); i++)
                // {
                //     std::cout << "Waypoint " << i << ": (" << path_grid[i][0] << ", " << path_grid[i][1] << ", " << path_grid[i][2] << ")" << std::endl;
                // }

                std::cout << "Pomp 25 JPS planning successful." << std::endl;
            }
            else
            {
                // pomp_find = false;
                std::cout << "Pomp JPS did not find a valid path." << std::endl;
            }
        }




        std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
        grid_map->insertPcd(env->points_array);
        {
            vec_Vec3f path_grid;
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
            bool valid_grid_jps_path = planner_ptr->plan(start, goal, 1, true);
            // auto end_plan_time = std::chrono::high_resolution_clock::now();
            // duration_grid_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_grid_jps_path)
            {
                path_grid = planner_ptr->getPath();

                // current_gird_path_dist = get_distance(path_grid);

                grid_success++;
                // std::cout << "Grid JPS found path with " << path_grid.size() << " waypoints." << std::endl;
                // for (int i = 0; i < path_grid.size(); i++)
                // {
                //     std::cout << "Waypoint " << i << ": (" << path_grid[i][0] << ", " << path_grid[i][1] << ", " << path_grid[i][2] << ")" << std::endl;
                // }

                std::cout << "Grid JPS planning successful." << std::endl;
            }
            else
            {

                std::cout << "Grid JPS did not find a valid path." << std::endl;
            }
        }
        geom_env *current_frame = frames->get_frame_ptr(frame_idx);

        // if (grid_find && pomp_find)
        // {
        //     grid_path_length = grid_path_length + current_gird_path_dist;
        //     pomp_path_length = pomp_path_length + current_pomp_path_dist;
        //     both_find++;
        // }

        // pomp_path_length = pomp_path_length + path_length;
        // grid_path_length= grid_path_length +
        std::cout << "frame id: " << frame_idx << std::endl;
        if (current_frame == nullptr || frame_idx == 1000)
        {

            std::cout << "frame amount: " << frame_idx << std::endl;
            std::cout << "pomp 95 success: " << pomp_success_95 << std::endl;
            std::cout << "pomp 75 success: " << pomp_success_75 << std::endl;
            std::cout << "pomp 50 success: " << pomp_success_50 << std::endl;
            std::cout << "pomp 25 success: " << pomp_success_25 << std::endl;
            std::cout << "grid success: " << grid_success << std::endl;
            // std::cout << "pomp average path distance: " << pomp_path_length / both_find << std::endl;
            // std::cout << "grid average path distance: " << grid_path_length / both_find << std::endl;

            break;
        }
        // pdPublisher->read_frame(current_frame);
        // pdPublisher->publish();

        frames->next_frame();

        frame_idx++;
        // rate.sleep();
        // ros::spinOnce();
    }

    // ros::spin();

    return 0;
}