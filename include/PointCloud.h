#pragma once
#include <vector> 
#include <string>

struct Point3D 
{
    float x;
    float y;
    float z;
};

class PointCloud
{
    std::vector<Point3D> points;

public:
    
    void addPoint(float x, float y, float z)
    {
        points.push_back({x, y, z});
    }

    size_t size() const
    {
        return points.size();
    }

    bool loadData(const std::string& filepath);
};