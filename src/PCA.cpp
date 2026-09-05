#include "PCA.hpp"
#include <iostream>

OBB findOBB(const std::vector<Point3D> &clusterCloud)
{
    // quick eigen test

    OBB result;
    Eigen::Matrix3f testMatrix = Eigen::Matrix3f::Identity();

    std::cout << "Eigen dziala :)\n" << testMatrix << "\n" << std::endl;

    return result;
}