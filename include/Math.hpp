#pragma once
#include "PointCloud.hpp"
#include <cmath>

namespace Math3D
{

// helper function for KdTree algorithms
inline float distSquared(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

inline Point3D subVectors(const Point3D &a, const Point3D &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Point3D crossProduct(const Point3D &v1, const Point3D &v2)
{
    return {(v1.y * v2.z) - (v1.z * v2.y), (v1.z * v2.x) - (v1.x * v2.z),
            (v1.x * v2.y) - (v1.y * v2.x)};
}

} // namespace Math3D