#include "PCA.hpp"
#include <iostream>

OBB findOBB(const std::vector<Point3D> &clusterCloud)
{
    OBB result;

    /*
    // quick eigen test
    Eigen::Matrix3f testMatrix = Eigen::Matrix3f::Identity();
    std::cout << "Eigen dziala :)\n" << testMatrix << "\n" << std::endl;
    */

    // 1 find object's centroid
    int x = 0.0f, y = 0.0f, z = 0.0f;
    for (const auto &point : clusterCloud)
    {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    x /= static_cast<float>(clusterCloud.size());
    y /= static_cast<float>(clusterCloud.size());
    z /= static_cast<float>(clusterCloud.size());
    Eigen::Vector3f centroid(x, y, z);

    // 2 wycentrujemy chmure na (0,0,0) i zbudujemy cov matrix
    Eigen::Matrix3f covMat = Eigen::Matrix3f::Zero();
    for (const auto &point : clusterCloud)
    {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f diff = pt - centroid;
        covMat += diff * diff.transpose();
    }

    // 3 wektory wlasne

    // 4 final boudning box

    return result;
}