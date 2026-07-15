#pragma once
#include <vector>
#include <string>

struct Point3D {
    float x;
    float y;
    float z;
};

class PointCloud {
public:
    std::vector<Point3D> points;
    bool loadFromPLY(const std::string& filename);
};