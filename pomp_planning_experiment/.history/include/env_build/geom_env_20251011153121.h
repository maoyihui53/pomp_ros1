#pragma once
#include<env_build/animation.h>

// struct geom_frames
// {
//     pcl::PointCloud<pcl::PointXYZRGB> cloud_balls;
//     pcl::PointCloud<pcl::PointXYZRGB> cloud_cones;
//     pcl::PointCloud<pcl::PointXYZRGB> cloud_boxes;

//     geom_frames()
//     {
//         cloud_balls.reserve(300000);
//         cloud_cones.reserve(300000);
//         cloud_boxes.reserve(300000);
//     };

    

// };

// ----------------- helpers -----------------
template <class T>
static inline T clamp(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

inline pcl::PointXYZ make_pt(double x, double y, double z)
{
    pcl::PointXYZ p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
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

static void sample_sphere_surface(pcl::PointCloud<pcl::PointXYZ> &cloud,
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
        cloud.emplace_back(make_pt(x, y, z));
    }
}

// center (cx,cy,cz), half-size s  -> cube of side 2s
static void sample_boxes_surface(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                 std::mt19937 &gen,
                                 double cx, double cy, double cz,
                                 double s,
                                 double surface_density_pts_per_m2)
{
    // Total surface area of cube = 6 * (2s * 2s) = 24 s^2
    const double Atotal = 24.0 * s * s;
    const int N = std::max(200, (int)std::lround(surface_density_pts_per_m2 * Atotal));
    cloud.reserve(cloud.size() + N);

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // One uniform color for all boxes (blue). If you want per-box colors, make an overload with (R,G,B).
    const uint8_t color[3] = {0, 0, 255};

    // Precompute extents
    const double xmin = cx - s, xmax = cx + s;
    const double ymin = cy - s, ymax = cy + s;
    const double zmin = cz - s, zmax = cz + s;

    for (int i = 0; i < N; ++i)
    {
        // All faces have equal area, so pick a face uniformly from 0..5
        const int face = (int)std::floor(u01(gen) * 6.0);

        const double u = u01(gen);
        const double v = u01(gen);

        double x, y, z;
        switch (face)
        {
        case 0: // x = xmin
            x = xmin;
            y = ymin + u * (ymax - ymin);
            z = zmin + v * (zmax - zmin);
            break;
        case 1: // x = xmax
            x = xmax;
            y = ymin + u * (ymax - ymin);
            z = zmin + v * (zmax - zmin);
            break;
        case 2: // y = ymin
            x = xmin + u * (xmax - xmin);
            y = ymin;
            z = zmin + v * (zmax - zmin);
            break;
        case 3: // y = ymax
            x = xmin + u * (xmax - xmin);
            y = ymax;
            z = zmin + v * (zmax - zmin);
            break;
        case 4: // z = zmin
            x = xmin + u * (xmax - xmin);
            y = ymin + v * (ymax - ymin);
            z = zmin;
            break;
        default: // 5: z = zmax
            x = xmin + u * (xmax - xmin);
            y = ymin + v * (ymax - ymin);
            z = zmax;
            break;
        }

        
        cloud.emplace_back(make_pt(x, y, z));
        
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
        cloud.emplace_back(make_pt(x, y, z));
    }
}

void build_env_pointcloud(ros::NodeHandle &nh, pcl::PointCloud<pcl::PointXYZ> &cloud, int seed_ = 42)
{

    // 3) optional knobs
    double surface_density = 100.0; // points per m^2 on surfaces
    int seed = seed_;
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
    cloud->reserve(1000000); // rough prealloc

    // 7) balls
    for (int i = 0; i < balls_param.num_balls; ++i)
    {
        double r = rx_ball(gen);
        // keep inside AABB with margin r
        double x = urand(gen, r, Lx - r);
        double y = urand(gen, r, Ly - r);
        double z = urand(gen, r, Lz - r);
        sample_sphere_surface(cloud, gen, x, y, z, r, surface_density);
    }

    // 7) boxes
    for (int i = 0; i < boxes_param.num_boxes; ++i)
    {
        double s = rx_boxes_s(gen);
        // keep inside AABB with margin r
        double x = urand(gen, s, Lx - s);
        double y = urand(gen, s, Ly - s);
        double z = urand(gen, s, Lz - s);
        sample_boxes_surface(cloud, gen, x, y, z, s, surface_density);
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
        sample_cone_surface(cloud, gen, x, y, z, r, h, surface_density);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    // return cloud;
}