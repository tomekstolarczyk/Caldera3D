#pragma once

#include "KdTree.hpp"
#include "PointCloud.hpp"
#include <vector>

std::vector<std::vector<Point3D>> euclideanClusterization(const std::vector<Point3D> &pointCloud,
                                                          const KdTree &kdtree,
                                                          float maxPointDistance);
