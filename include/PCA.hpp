#pragma once

#include "PointCloud.hpp"
#include <Eigen/Dense> // for eigenvector math
#include <vector>

// Oriented Bounding Box
struct OBB
{
    float height;
    float width;
    float length;
    Point3D centroid;
    Eigen::Matrix3f rotationAxes; // save eigenvectors for future grasp pose estimation
};

OBB findOBB(const std::vector<Point3D> &clusterCloud);