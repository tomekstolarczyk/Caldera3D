#include "PCA.hpp"
#include <iostream>

OBB findOBB(const std::vector<Point3D> &clusterCloud)
{
    OBB result;

    if (clusterCloud.empty())
    {
        return result;
    }

    /*
    // quick eigen test
    Eigen::Matrix3f testMatrix = Eigen::Matrix3f::Identity();
    std::cout << "Eigen dziala :)\n" << testMatrix << "\n" << std::endl;
    */

    // 1 find object's centroid
    float x = 0.0f, y = 0.0f, z = 0.0f;
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
    covMat /= static_cast<float>(clusterCloud.size());

    // 3 wektory wlasne
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covMat);
    Eigen::Vector3f height = solver.eigenvectors().col(0); // shortest
    Eigen::Vector3f width = solver.eigenvectors().col(1);
    Eigen::Vector3f length = solver.eigenvectors().col(2); // longest

    // 4 find the final boudning box's extremes
    // mamy same kierunki rozrzutu najwiekszego - teraz trzeba znalezc jego faktyczne granice
    float minH = std::numeric_limits<float>::max(), maxH = std::numeric_limits<float>::lowest();
    float minW = std::numeric_limits<float>::max(), maxW = std::numeric_limits<float>::lowest();
    float minL = std::numeric_limits<float>::max(), maxL = std::numeric_limits<float>::lowest();
    for (const auto &point : clusterCloud)
    {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f centered = pt - centroid;
        // terz rzutujemy ten wektor punktow na kazdy z 3 wektorow wlasnych
        float projH = centered.dot(height);
        float projW = centered.dot(width);
        float projL = centered.dot(length);
        // aktualizujemy extrema
        minH = std::min(minH, projH);
        maxH = std::max(maxH, projH);
        minW = std::min(minW, projW);
        maxW = std::max(maxW, projW);
        minL = std::min(minL, projL);
        maxL = std::max(maxL, projL);
    }

    // 5 result
    result.height = maxH - minH;
    result.width = maxW - minW;
    result.length = maxL - minL;
    result.centroid = {centroid[0], centroid[1], centroid[2]};
    result.rotationAxes.col(0) = height;
    result.rotationAxes.col(1) = width;
    result.rotationAxes.col(2) = length;

    return result;
}