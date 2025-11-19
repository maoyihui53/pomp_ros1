#pragma once
#include <env_build/animation.h>

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

// center (cx,cy,cz), half-size s  -> cube of side 2s
static void sample_boxes_surface(std::unique_ptr<geom_env> &cloud,
                                 std::mt19937 &gen,
                                 double cx, double cy, double cz,
                                 double s,
                                 double surface_density_pts_per_m2)
{
    // Total surface area of cube = 6 * (2s * 2s) = 24 s^2
    const double Atotal = 24.0 * s * s;
    const int N = std::max(200, (int)std::lround(surface_density_pts_per_m2 * Atotal));
    Eigen::Vector3d center(cx, cy, cz);

    geom cube(N);

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

        Eigen::Vector3d pts(x, y, z);

        cube.push_back(pts);

        // cloud->push_back(pts);

        // cloud.emplace_back(make_pt(x, y, z));
    }
    cloud->set_centers(center);
    cloud->set_half_size(s);
    cloud->push_back(cube);
}

// Sample points on the lateral surface of a right circular cone standing on z = z0

void build_env_pointcloud(ros::NodeHandle &nh, std::unique_ptr<geom_env> &cloud, int seed_ = 42)
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
    std::uniform_real_distribution<double> rx_boxes_s(boxes_param.size_min_max[0], boxes_param.size_min_max[1]);

    // 6) output cloud
    // auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // cloud->reserve(1000000); // rough prealloc

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

    // cloud->width = cloud->size();
    // cloud->height = 1;
    // cloud->is_dense = false;
    // return cloud;
}