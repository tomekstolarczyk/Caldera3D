#pragma once

#include "PointCloud.hpp"

std::vector<std::vector<Point3D>>
everythingAboveTheTable(const std::vector<std::vector<Point3D>> &dataClusters, float ATable,
                        float BTable, float CTable, float DTable);
