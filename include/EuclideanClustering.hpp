#pragma once

#include "KdTree.hpp"
#include "Math.hpp"
#include "PointCloud.hpp"

std::vector<std::vector<Point3D>> euclideanClusterization(const std::vector<Point3D> &pointCloud,
                                                          float maxPointDistance);