#pragma once
#include <string>
#include <vector>

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
    // setters

    void addPoint(float x, float y, float z)
    {
        points.push_back({x, y, z});
    }

    void setPoints(const std::vector<Point3D> &newPoints)
    {
        points = newPoints;
    }

    // getters
    const std::vector<Point3D> &getPoints() const
    {
        return points;
    }

    size_t size() const
    {
        return points.size();
    }

    bool loadData(const std::string &filepath);
    bool saveData(const std::string &filepath) const;
};