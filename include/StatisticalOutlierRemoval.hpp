// statistical-outlier-removal filter

#pragma once

#include "KdTree.hpp"
#include "PointCloud.hpp"
#include <vector>

struct MeanStdDev
{
    double mean;
    double stddev; // sample standard deviation (n - 1)
};

// two-pass mean / standard deviation accumulated in double
MeanStdDev computeMeanStdDev(const std::vector<float> &values);

std::vector<int> filterSOR(const std::vector<Point3D> &points, const KdTree &tree, int k,
                           float alpha);
