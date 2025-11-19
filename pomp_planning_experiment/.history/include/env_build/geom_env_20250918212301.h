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