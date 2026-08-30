#pragma once
#include "PointCloud.hpp"
#include <utility> // std::pair
#include <vector>

struct ransacSinglePlaneResult
{
    std::vector<Point3D> inliers;
    std::vector<Point3D> outliers;
    float A, B, C, D;
};

struct ransacFinalResult
{
    std::vector<Point3D> objects;         // to co idzie ostatecznie do klastracji
    int planesFound;                      // liczba znalezionych plaszyzn
    float tableA, tableB, tableC, tableD; // rownanie blatu stolu
};

// znajduje jedna plaszyzne
ransacSinglePlaneResult ransacFindPlane(const std::vector<Point3D> &cloud, int k, float threshold);

// znajduje wszystkie wielkie plaszyzny - podloga, stol, sciany
ransacFinalResult ransacBulkSceneSegmentation(const std::vector<Point3D> &cloud, int k,
                                              float threshold, int minPlaneSize);