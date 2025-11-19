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

static void sample_sphere_surface(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                  std::mt19937 &gen,
                                  double cx, double cy, double cz,
                                  double r, double surface_density_pts_per_m2)
{
    const double area = 4.0 * M_PI * r * r;
    int N = std::max(64, (int)std::round(surface_density_pts_per_m2 * area));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_real_distribution<double> uphi(0.0, 2.0 * M_PI);

    for (int i = 0; i < N; ++i)
    {
        // Uniform on sphere: cosθ ~ U[-1,1], φ ~ U[0,2π)
        double zeta = 2.0 * u01(gen) - 1.0; // cosθ
        double phi = uphi(gen);
        double sinT = std::sqrt(std::max(0.0, 1.0 - zeta * zeta));
        double x = cx + r * sinT * std::cos(phi);
        double y = cy + r * sinT * std::sin(phi);
        double z = cz + r * zeta;
        cloud.emplace_back((float)x, (float)y, (float)z);
    }
}

// Sample points on the lateral surface of a right circular cone standing on z = z0
static void sample_cone_surface(pcl::PointCloud<pcl::PointXYZ> &cloud,
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

    for (int i = 0; i < N; ++i)
    {
        // parameter t along slant from base (t=0) to tip (t=1)
        double t = u01(gen);
        double phi = uphi(gen);
        double rt = (1.0 - t) * r; // radius at that slice
        double z = z0 + t * h;
        double x = cx + rt * std::cos(phi);
        double y = cy + rt * std::sin(phi);
        cloud.emplace_back((float)x, (float)y, (float)z);
    }
}

// Cylindrical arc wall: sweep angle [ang0, ang1] (radians) in XY, extruded height h along +z from z0
static void sample_cylindrical_arc(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                   std::mt19937 &gen,
                                   double cx, double cy, double z0,
                                   double r, double h,
                                   double ang0_rad, double ang1_rad,
                                   double surface_density_pts_per_m2)
{
    double sweep = std::fabs(ang1_rad - ang0_rad);
    const double area = sweep * r * h; // area of curved wall patch
    int N = std::max(100, (int)std::round(surface_density_pts_per_m2 * area));

    std::uniform_real_distribution<double> uang(std::min(ang0_rad, ang1_rad), std::max(ang0_rad, ang1_rad));
    std::uniform_real_distribution<double> uz(z0, z0 + h);

    for (int i = 0; i < N; ++i)
    {
        double a = uang(gen);
        double z = uz(gen);
        double x = cx + r * std::cos(a);
        double y = cy + r * std::sin(a);
        cloud.emplace_back((float)x, (float)y, (float)z);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr build_env_pointcloud(ros::NodeHandle &nh)
{

    // 3) optional knobs
    double surface_density = 300.0; // points per m^2 on surfaces
    int seed = 42;

    std::mt19937 gen(seed);

    // 5) distributions
    std::uniform_real_distribution<double> rx_ball(balls_param.radius_min_max[0], balls_param.radius_min_max[1]);
    std::uniform_real_distribution<double> rx_cone_r(cones_param.radius_min_max[0], cones_param.radius_min_max[1]);
    std::uniform_real_distribution<double> rx_cone_h(cones_param.height_min_max[0], cones_param.height_min_max[1]);
    std::uniform_real_distribution<double> rx_arc_r(arcs_param.radius_min_max[0], arcs_param.radius_min_max[1]);
    std::uniform_real_distribution<double> rx_arc_h(arcs_param.height_min_max[0], arcs_param.height_min_max[1]);

    double ang0_deg = arcs_param.angle_min_max[0];
    double ang1_deg = arcs_param.angle_min_max[1];
    double ang0_rad = ang0_deg * M_PI / 180.0;
    double ang1_rad = ang1_deg * M_PI / 180.0;

    // 6) output cloud
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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

    // 8) cones (upright on ground z=0)
    for (int i = 0; i < cones_param.num_cones; ++i)
    {
        double r = rx_cone_r(gen);
        double h = rx_cone_h(gen);
        if (h > Lz)
            h = Lz * 0.9; // clamp
        double z0 = 0.0;  // base at ground
        double x = urand(gen, r, Lx - r);
        double y = urand(gen, r, Ly - r);
        sample_cone_surface(*cloud, gen, x, y, z0, r, h, surface_density);
    }

    // 9) arcs (cylindrical wall segment extruded along z)
    for (int i = 0; i < arcs_param.num_arcs; ++i)
    {
        double r = rx_arc_r(gen);
        double h = rx_arc_h(gen);
        if (h > Lz)
            h = Lz * 0.9;
        double z0 = urand(gen, 0.0, Lz - h);
        // center must keep the radius inside
        double cx = urand(gen, r, Lx - r);
        double cy = urand(gen, r, Ly - r);
        sample_cylindrical_arc(*cloud, gen, cx, cy, z0, r, h, ang0_rad, ang1_rad, surface_density);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}