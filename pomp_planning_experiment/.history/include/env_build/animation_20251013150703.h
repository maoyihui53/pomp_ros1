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

public:
    geom_env()
    {
        idx = 0;
        cloud_boxes.reserve(boxes_param.num_boxes);
        centers.reserve(boxes_param.num_boxes);
        half_sizes.reserve(boxes_param.num_boxes);
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

    void geom_move_in_one_frame()
    {
        for (auto &b : cloud_boxes)
        {
            for (int i = 0; i < b.amount(); i++)
            {
                double dx = urand(gen, speed_x[0], speed_x[1]);
                double dy = urand(gen, speed_y[0], speed_y[1]);
                double dz = urand(gen, speed_z[0], speed_z[1]);

                Eigen::Vector3d pos = b.get_geom_pos().row(i);
                pos(0) += dx;
                pos(1) += dy;
                pos(2) += dz;

                // check boundary
                if (pos(0) < -map_size[0] / 2.f || pos(0) > map_size[0] / 2.f)
                    pos(0) -= dx; // revert
                if (pos(1) < -map_size[1] / 2.f || pos(1) > map_size[1] / 2.f)
                    pos(1) -= dy; // revert
                if (pos(2) < -map_size[2] / 2.f || pos(2) > map_size[2] / 2.f)
                    pos(2) -= dz; // revert

                b.get_geom_pos().row(i) = pos.transpose();
            }
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
        speed_x[0] = boxes_param.speed[0] - boxes_param.speed[3];
        speed_x[1] = boxes_param.speed[0] + boxes_param.speed[3];

        speed_y[0] = boxes_param.speed[1] - boxes_param.speed[3];
        speed_y[1] = boxes_param.speed[1] + boxes_param.speed[3];

        speed_z[0] = boxes_param.speed[2] - boxes_param.speed[3];
        speed_z[1] = boxes_param.speed[2] + boxes_param.speed[3];

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
    inline std::array<double, 2> unit2D_from_deg(double deg)
    {
        double t = deg2rad(deg);
        return {std::cos(t), std::sin(t)};
    }

    void next_frame()
    {

        int idx = frames.size(); // next_idx
        if (idx > 0)
        {
            geom_env *prev_geom_env = get_frame_ptr(idx - 1);
            std::unique_ptr<geom_env> new_geom_env = std::make_unique<geom_env>(*prev_geom_env);
        }
    }
};