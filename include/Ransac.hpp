#pragma once
#include "PointCloud.hpp"

std::vector<Point3D> rANSAC(const std::vector<Point3D> &cloud, int k);