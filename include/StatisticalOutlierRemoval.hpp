// statistical-outlier-removal filter

#pragma once

#include "KdTree.hpp"
#include "PointCloud.hpp"
#include <vector>

std::vector<int> filterSOR(const KdTree &tree, int k, float alpha);
