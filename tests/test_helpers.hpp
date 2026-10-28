#pragma once
#include "PointCloud.hpp"
#include <random>
#include <vector>

// deterministic random cloud in a box - fixed seed so failures are reproducible
inline std::vector<Point3D> randomCloud(size_t n, float lo, float hi, unsigned seed = 42)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(lo, hi);
    std::vector<Point3D> cloud(n);
    for (auto &p : cloud)
    {
        p = {dist(rng), dist(rng), dist(rng)};
    }
    return cloud;
}
