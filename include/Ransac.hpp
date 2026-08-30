#pragma once
#include "PointCloud.hpp"
#include <utility> // std::pair
#include <vector>

struct ransacResult
{
    std::vector<Point3D> inliers;
    std::vector<Point3D> outliers;
    float A, B, C, D;
};

// znajduje jedna plaszyzne
ransacResult findPlanes(const std::vector<Point3D> &cloud, int k, float threshold);

// znajduje wszystkie wielkie plaszyzny - podloga, stol, sciany
std::vector<ransacResult> ransacBulk(const std::vector<Point3D> &cloud, int k, float threshold,
                                     int minPlaneSize);