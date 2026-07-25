#pragma once
#include "PointCloud.hpp"

struct voxelBrick
{
    int count = 0;
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
};

std::vector<Point3D> voxelizePointCloud(const std::vector<Point3D> &pointCloud, float voxelSize);