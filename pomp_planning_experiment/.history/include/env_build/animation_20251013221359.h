#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <utils/util.h>
#include <Eigen/Dense>
#include <vector>

#include <stdexcept>
constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }

static std::vector<std::array<double, 3>> unit_dir_vector;
class geom
{

    Eigen::MatrixX3d geom_pos;
    int idx;

public:
    geom(int N)
    {
        geom_pos.resize(N, 3);
        idx = 0;
    }

    void push_back(const Eigen::Vector3d &pos)
    {
        geom_pos.row(idx) = pos.transpose(); //
        idx++;
    }

    Eigen::MatrixX3d get_geom_pos()
    {
        return geom_pos;
    }

    int amount()
    {
        return idx;
    }


    
};

class geom_env
{
    int idx;
    std::vector<geom> cloud_boxes;
    std::vector<Eigen::Vector3d> centers;
    std::vector<double> half_sizes;
    std::vector<double> dir;

public:
    geom_env()
    {
        idx = 0;
        cloud_boxes.reserve(boxes_param.num_boxes);
        centers.reserve(boxes_param.num_boxes);
        half_sizes.reserve(boxes_param.num_boxes);
        set_dir();
    }

    geom_env(const geom_env &other)
        : idx(other.idx),
          cloud_boxes(other.cloud_boxes),
          centers(other.centers),
          half_sizes(other.half_sizes) {}

    void set_centers(const Eigen::Vector3d &center)
    {
        centers.emplace_back(center);
    }

    void set_half_size(double hs)
    {
        half_sizes.emplace_back(hs);
    }

    void set_dir()
    {
        dir.resize(boxes_param.num_boxes);
    }

    void push_back(const geom &cube)
    {
        cloud_boxes.emplace_back(cube);
    }

    void print_info()
    {
        std::cout << "boxes amount: " << cloud_boxes.size() << std::endl;
        std::cout << "center amount: " << centers.size() << std::endl;
        std::cout << "half_size: " << half_sizes.size() << std::endl;

        std::cout << "points amount: " << pd_amount() << std::endl;
    }

    int pd_amount()
    {
        int idx_ = 0;
        for (auto &b : cloud_boxes)
        {
            idx_ = idx_ + b.amount();
        }
        idx = idx_;
        return idx;
    }

    std::vector<geom> get_boxes()
    {
        return cloud_boxes;
    }

    static inline double urand(std::mt19937 &gen, double a, double b)
    {
        std::uniform_real_distribution<double> dist(a, b);
        return dist(gen);
    }

    int get_boxes_amount()
    {
        return cloud_boxes.size();
    }

    void move_one_step()
    {
        for (int i = 0; i < cloud_boxes.size(); i++)
        {
            geom cube = cloud_boxes[i];

        }
    }
};

class animation_frames
{

private:
    std::vector<std::unique_ptr<geom_env>> frames;
    double speed_x[2];
    double speed_y[2];
    double speed_z[2];

public:
    animation_frames(int frame_num = 1000)
    {
        frames.reserve(frame_num);
        std::cout << "speed min " << boxes_param.speed_min << std::endl;
        std::cout << "speed max " << boxes_param.speed_max << std::endl;

        // speed_x[0] = boxes_param.speed[0] - boxes_param.speed[3];
        // speed_x[1] = boxes_param.speed[0] + boxes_param.speed[3];

        // speed_y[0] = boxes_param.speed[1] - boxes_param.speed[3];
        // speed_y[1] = boxes_param.speed[1] + boxes_param.speed[3];

        // speed_z[0] = boxes_param.speed[2] - boxes_param.speed[3];
        // speed_z[1] = boxes_param.speed[2] + boxes_param.speed[3];

        // std::unique_ptr<geom_env> cubes_ = std::make_unique<geom_env>();

        // build_env_pointcloud(cubes_);
        // frames.emplace_back(cubes_);
        // frames[0].print_info();
    }
    void push_back(std::unique_ptr<geom_env> g)
    {
        frames.emplace_back(std::move(g)); // or: frames.push_back(std::move(g));
    }

    void print_info(int idx = 0)
    {
        frames[idx]->print_info();
    }

    geom_env *get_frame_ptr(std::size_t idx) noexcept
    {
        if (idx >= frames.size())
            return nullptr;
        return frames[idx].get(); // frames is vector<unique_ptr<geom_env>>
    }

    int pd_amount_in_frame(int i)
    {
        return frames[i]->pd_amount();
    }

    // Thread-safe-ish generator per thread
    inline std::mt19937 &rng()
    {
        thread_local std::mt19937 gen{std::random_device{}()};
        return gen;
    }

    // 1) Given a heading in degrees [0, 360) -> 2D unit vector (cos θ, sin θ)
    inline std::array<double, 3> random_unit3D()
    {
        static thread_local std::uniform_real_distribution<double> Uphi(0.0, 2.0 * kPi);
        static thread_local std::uniform_real_distribution<double> Uz(-1.0, 1.0);
        double z = Uz(rng());              // cos(theta) ~ Uniform[-1,1]
        double phi = Uphi(rng());          // azimuth
        double r = std::sqrt(1.0 - z * z); // radius in XY
        double x = r * std::cos(phi);
        double y = r * std::sin(phi);
        return {x, y, z};
    }

    void next_frame()
    {
        for (int i = 0; i < get_frame_ptr(0)->get_boxes_amount(); i++)
        {
            std::array<double, 3> unit_v = random_unit3D();
            unit_dir_vector.push_back(unit_v);

            // std::cout << "unit_dir_vector: " << unit_dir_vector[i][0] << " "
            //           << unit_dir_vector[i][1] << " "
            //           << unit_dir_vector[i][2] << std::endl;
        }

        int idx = frames.size(); // next_idx
        if (idx > 0)
        {
            geom_env *prev_geom_env = get_frame_ptr(idx - 1);
            std::unique_ptr<geom_env> new_geom_env = std::make_unique<geom_env>(*prev_geom_env);
        }
    }
};