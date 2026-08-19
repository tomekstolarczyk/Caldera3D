#pragma once
#include "PointCloud.hpp"

struct BoundingBox
{
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
};

std::vector<Point3D> passThroughFilter(const BoundingBox &box, const std::vector<Point3D> &points);