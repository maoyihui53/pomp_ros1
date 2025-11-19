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

// ----------------- helpers -----------------
template <class T>
static inline T clamp(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

inline pcl::PointXYZRGB make_pt(double x, double y, double z,
                                uint8_t color[])
{
    pcl::PointXYZRGB p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    p.r = color[0];
    p.g = color[1];
    p.b = color[2];
    return p;
}
static inline double urand(std::mt19937 &gen, double a, double b)
{
    std::uniform_real_distribution<double> dist(a, b);
    return dist(gen);
}
static inline int irand(std::mt19937 &gen, int a, int b)
{
    std::uniform_int_distribution<int> dist(a, b);
    return dist(gen);
}

static void sample_sphere_surface(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                  std::mt19937 &gen,
                                  double cx, double cy, double cz,
                                  double r, double surface_density_pts_per_m2)
{
    const double area = 4.0 * M_PI * r * r;
    int N = std::max(64, (int)std::round(surface_density_pts_per_m2 * area));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_real_distribution<double> uphi(0.0, 2.0 * M_PI);

    uint8_t color[3] = {255, 0, 0};

    for (int i = 0; i < N; ++i)
    {
        // Uniform on sphere: cosθ ~ U[-1,1], φ ~ U[0,2π)
        double zeta = 2.0 * u01(gen) - 1.0; // cosθ
        double phi = uphi(gen);
        double sinT = std::sqrt(std::max(0.0, 1.0 - zeta * zeta));
        double x = cx + r * sinT * std::cos(phi);
        double y = cy + r * sinT * std::sin(phi);
        double z = cz + r * zeta;
        cloud.emplace_back(make_pt(x, y, z, color));
    }
}

// Sample points on the lateral surface of a right circular cone standing on z = z0
static void sample_cone_surface(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                std::mt19937 &gen,
                                double cx, double cy, double z0,
                                double r, double h,
                                double surface_density_pts_per_m2)
{
    const double s = std::sqrt(r * r + h * h); // slant height
    const double area = M_PI * r * s;          // lateral area
    int N = std::max(100, (int)std::round(surface_density_pts_per_m2 * area));

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_real_distribution<double> uphi(0.0, 2.0 * M_PI);
    uint8_t color[3] = {0, 255, 0};
    for (int i = 0; i < N; ++i)
    {
        // parameter t along slant from base (t=0) to tip (t=1)
        double t = u01(gen);
        double phi = uphi(gen);
        double rt = (1.0 - t) * r; // radius at that slice
        double z = z0 + t * h;
        double x = cx + rt * std::cos(phi);
        double y = cy + rt * std::sin(phi);
        cloud.emplace_back(make_pt(x, y, z, color));
    }
}

void build_env_pointcloud(ros::NodeHandle &nh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{

    // 3) optional knobs
    double surface_density = 50.0; // points per m^2 on surfaces
    int seed = 42;
    double Lx = map_size[0];
    double Ly = map_size[1];
    double Lz = map_size[2];

    std::cout << "Lx: " << Lx << ", Ly: " << Ly << ", Lz: " << Lz << std::endl;

    std::mt19937 gen(seed);

    // 5) distributions
    std::uniform_real_distribution<double> rx_ball(balls_param.radius_min_max[0], balls_param.radius_min_max[1]);
    std::uniform_real_distribution<double> rx_cone_r(cones_param.radius_min_max[0], cones_param.radius_min_max[1]);
    std::uniform_real_distribution<double> rx_cone_h(cones_param.height_min_max[0], cones_param.height_min_max[1]);
    std::uniform_real_distribution<double> rx_boxes_s(boxes_param.size_min_max[0], boxes_param.size_min_max[1]);

    // 6) output cloud
    // auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(500000); // rough prealloc

    // 7) balls
    for (int i = 0; i < balls_param.num_balls; ++i)
    {
        double r = rx_ball(gen);
        // keep inside AABB with margin r
        double x = urand(gen, r, Lx - r);
        double y = urand(gen, r, Ly - r);
        double z = urand(gen, r, Lz - r);
        sample_sphere_surface(*cloud, gen, x, y, z, r, surface_density);
    }

    // 7) balls
    for (int i = 0; i < boxes_param.num_boxes; ++i)
    {
        double s = rx_boxes_s(gen);
        // keep inside AABB with margin r
        double x = urand(gen, s, Lx - s);
        double y = urand(gen, s, Ly - s);
        double z = urand(gen, s, Lz - s);
        sample_boxes_surface(*cloud, gen, x, y, z, s, surface_density);
    }

    // 8) cones (upright on ground z=0)
    for (int i = 0; i < cones_param.num_cones; ++i)
    {
        double r = rx_cone_r(gen);
        double h = rx_cone_h(gen);
        if (h > Lz)
            h = Lz * 0.9; // clamp
        double z = urand(gen, h, Lz - h);
        double x = urand(gen, r, Lx - r);
        double y = urand(gen, r, Ly - r);
        sample_cone_surface(*cloud, gen, x, y, z, r, h, surface_density);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    // return cloud;
}