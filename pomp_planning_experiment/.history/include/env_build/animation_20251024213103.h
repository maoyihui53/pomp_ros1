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

    void move_step(const std::array<double, 3> &step, int dir)
    {

        Eigen::Vector3d stp(dir * step[0], dir * step[1], dir * step[2]);
        int N = geom_pos.rows();
        for (int i = 0; i < N; i++)
            geom_pos.row(i) = geom_pos.row(i) + stp.transpose();
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
          half_sizes(other.half_sizes)
    {

        set_dir();
    }

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
        dir.resize(boxes_param.num_boxes, 1);
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
    // double doublerand(std::mt19937 &gen, double a, double b)
    // {
    //     std::uniform_real_distribution<double> dist(a, b);
    //     return dist(gen);
    // }
    // void move_one_step()
    // {
    //     double x_min = -map_size[0] / 2.f;
    //     double x_max = map_size[0] / 2.f;
    //     double y_min = -map_size[1] / 2.f;
    //     double y_max = map_size[1] / 2.f;
    //     double z_min = -map_size[2] / 2.f;
    //     double z_max = map_size[2] / 2.f;

    //     int seed = 42;

    //     std::mt19937 gen(seed);

    //     std::cout << "move_one_step()" << std::endl;
    //     std::cout << "cloud_boxes size: " << cloud_boxes.size() << std::endl;
    //     std::cout << "centers size: " << centers.size() << std::endl;
    //     std::cout << "half_sizes size: " << half_sizes.size() << std::endl;
    //     std::cout << "dir size: " << dir.size() << std::endl;
    //     for (int i = 0; i < cloud_boxes.size(); i++)
    //     {

    //         Eigen::Vector3d center = centers[i];
    //         double half_size = half_sizes[i];
    //         geom cube = cloud_boxes[i];
    //         double speed_scal = urand(gen, boxes_param.speed_min, boxes_param.speed_max);

    //         std::array<double, 3> speed = {unit_dir_vector[i][0] * speed_scal,
    //                                        unit_dir_vector[i][1] * speed_scal,
    //                                        unit_dir_vector[i][2] * speed_scal};
    //         if (center[0] - half_size <= x_min || center[0] + half_size >= x_max || center[1] - half_size <= y_min || center[1] + half_size >= y_max || center[2] - half_size <= z_min || center[2] + half_size >= z_max)
    //         {
    //             double close_x;
    //             double close_y;
    //             double close_z;

    //             if (std::abs(center[0] - x_min) < std::abs(center[0] - x_max))
    //                 close_x = x_min;
    //             else
    //                 close_x = x_max;

    //             if (std::abs(center[1] - y_min) < std::abs(center[1] - y_max))
    //                 close_y = y_min;
    //             else
    //                 close_y = y_max;

    //             if (std::abs(center[2] - z_min) < std::abs(center[2] - z_max))
    //                 close_z = z_min;
    //             else
    //                 close_z = z_max;

    //             if ((center[0] + dir[i] * speed[0] - close_x) * (center[0] + dir[i] * speed[0] - close_x) +
    //                     (center[1] + dir[i] * speed[1] - close_y) * (center[1] + dir[i] * speed[1] - close_y) +
    //                     (center[2] + dir[i] * speed[2] - close_z) * (center[2] + dir[i] * speed[2] - close_z) >
    //                 (center[0] - close_x) * (center[0] - close_x) +
    //                     (center[1] - close_y) * (center[1] - close_y) +
    //                     (center[2] - close_z) * (center[2] - close_z))

    //                 dir[i] = -dir[i];
    //         }

    //         cube.move_step(speed, dir[i]);

    //         center[0] = center[0] + dir[i] * speed[0];
    //         center[1] = center[1] + dir[i] * speed[1];
    //         center[2] = center[2] + dir[i] * speed[2];

    //         centers[i] = center;

    //         cloud_boxes[i] = cube;
    //     }
    // }
    void move_one_step()
    {
        const double x_min = -map_size[0] / 2.0;
        const double x_max = map_size[0] / 2.0;
        const double y_min = -map_size[1] / 2.0;
        const double y_max = map_size[1] / 2.0;
        const double z_min = -map_size[2] / 2.0;
        const double z_max = map_size[2] / 2.0;

        // 建议不要每次都用同一个 seed 重新构造引擎，否则每帧都一样
        static thread_local std::mt19937 gen(std::random_device{}());

        std::cout << "move_one_step()\n";
        std::cout << "cloud_boxes size: " << cloud_boxes.size() << "\n";
        std::cout << "centers size: " << centers.size() << "\n";
        std::cout << "half_sizes size: " << half_sizes.size() << "\n";
        std::cout << "dir size: " << dir.size() << "\n";

        auto reflect_axis = [](double &next_axis, double &dir_axis,
                               double lo, double hi)
        {
            // 只在越界时翻转该轴方向，并把穿透量镜像回去
            if (next_axis < lo)
            {
                const double pen = lo - next_axis; // 穿透量
                next_axis = lo + pen;              // 镜像回可达域
                dir_axis *= -1.0;                  // 只反转该轴方向
            }
            else if (next_axis > hi)
            {
                const double pen = next_axis - hi;
                next_axis = hi - pen;
                dir_axis *= -1.0;
            }
        };

        for (int i = 0; i < static_cast<int>(cloud_boxes.size()); ++i)
        {
            Eigen::Vector3d center = centers[i];
            const double half_size = half_sizes[i];
            geom cube = cloud_boxes[i];

            // 本步速度标量
            const double speed_scal = urand(gen, boxes_param.speed_min, boxes_param.speed_max);

            // 方向向量（单位）；将按轴可能被修改（反弹）
            Eigen::Vector3d dir_vec(unit_dir_vector[i][0],
                                    unit_dir_vector[i][1],
                                    unit_dir_vector[i][2]);

            // 本步位移向量
            Eigen::Vector3d v = dir_vec * speed_scal;

            // 预期下一位置（中心）
            Eigen::Vector3d next = center + v;

            // 每个轴的“可达范围”= [min+half_size, max-half_size]
            const double x_lo = x_min + half_size, x_hi = x_max - half_size;
            const double y_lo = y_min + half_size, y_hi = y_max - half_size;
            const double z_lo = z_min + half_size, z_hi = z_max - half_size;

            // 逐轴镜像反弹：只翻转越界轴
            reflect_axis(next[0], dir_vec[0], x_lo, x_hi);
            reflect_axis(next[1], dir_vec[1], y_lo, y_hi);
            reflect_axis(next[2], dir_vec[2], z_lo, z_hi);

            // 当前位置到下一位置的真实位移（包含反弹后的修正）
            const Eigen::Vector3d delta = next - center;

            // 更新保存的方向（单位化以避免数值漂移）
            if (dir_vec.norm() > 1e-12)
            {
                dir_vec.normalize();
            }
            else
            {
                // 极端情况下（几乎零向量）给个缺省方向，避免 NaN
                dir_vec = Eigen::Vector3d(1.0, 0.0, 0.0);
            }
            unit_dir_vector[i][0] = dir_vec[0];
            unit_dir_vector[i][1] = dir_vec[1];
            unit_dir_vector[i][2] = dir_vec[2];

            // 把 delta 作为这一步的“速度/位移”传给几何体
            // 如果 move_step 的第一个参数含义是“每轴位移量”，这么传最稳；
            // 第二个参数 dir 原先用作整体翻转，这里已不需要，给 1 即可（或在实现里忽略）。
            std::array<double, 3> step = {delta[0], delta[1], delta[2]};
            cube.move_step(step, /*dir=*/1);

            // 写回中心和几何体
            centers[i] = next;
            cloud_boxes[i] = cube;

            // 保留你原来的 dir[i]（如果别处还用到），但此函数里已不再依赖它
            // dir[i] = 1; // 可选：确保外部代码不会再用它做整体翻转
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

    void set_up()
    {

        for (int i = 0; i < get_frame_ptr(0)->get_boxes_amount(); i++)
        {
            std::array<double, 3> unit_v = random_unit3D();
            unit_dir_vector.push_back(unit_v);

            // std::cout << "unit_dir_vector: " << unit_dir_vector[i][0] << " "
            //           << unit_dir_vector[i][1] << " "
            //           << unit_dir_vector[i][2] << std::endl;
        }
    }

    Eigen::MatrixX3d get_pd_in_frame(int idx)
    {

        int pd_amount = pd_amount_in_frame(idx);
        Eigen::MatrixX3d pd;
        pd.resize(3, pd_amount);

        int box_amount = frames[idx]->get_boxes_amount();

        for (int i = 0; i < box_amount; i++)
        {
            Eigen::MatrixX3d pd_box = frames[idx]->get_boxes()[i].get_geom_pos();
        }
    }

    void next_frame()
    {

        int idx = frames.size(); // next_idx
        if (idx > 0)
        {
            geom_env *prev_geom_env = get_frame_ptr(idx - 1);
            std::unique_ptr<geom_env> new_geom_env = std::make_unique<geom_env>(*prev_geom_env);
            new_geom_env->move_one_step();
            push_back(std::move(new_geom_env));
        }
    }
};