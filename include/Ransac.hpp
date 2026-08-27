#pragma once
#include "PointCloud.hpp"

std::vector<Point3D> findPlanes(const std::vector<Point3D> &cloud, int k, float threshold);