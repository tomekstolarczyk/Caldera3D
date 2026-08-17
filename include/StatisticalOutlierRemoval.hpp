// statistical-outlier-removal filter

#pragma once

#include "KdTree.hpp"
#include "PointCloud.hpp"
#include <vector>

std::vector<int> filterSOR(const std::vector<Point3D> &points, const KdTree &tree, int k,
                           float alpha);
