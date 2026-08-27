#pragma once
#include "PointCloud.hpp"
#include <utility> // std::pair
#include <vector>

// para wektorow: {Inliers, Outliers}
std::pair<std::vector<Point3D>, std::vector<Point3D>> findPlanes(const std::vector<Point3D> &cloud,
                                                                 int k, float threshold);