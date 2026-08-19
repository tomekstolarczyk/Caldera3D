#include "PassThroughFilter.hpp"

std::vector<Point3D> passThroughFilter(const BoundingBox &box, const std::vector<Point3D> &points)
{
    std::vector<Point3D> filtered;
    filtered.reserve(points.size());
    // po prostu dla kazdego punktu przechodzimy przez niego i patrzymy czy lapie sie w pudelku
    // - tyle
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i].x <= box.maxX && points[i].x >= box.minX && points[i].y <= box.maxY &&
            points[i].y >= box.minY && points[i].z <= box.maxZ && points[i].z >= box.minZ)
        {
            filtered.push_back(points[i]);
        }
    }

    return filtered;
}