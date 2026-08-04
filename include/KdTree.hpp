#pragma once
#include "PointCloud.hpp"
#include <memory>

struct KdTreeNode
{
    Point3D point;
    int cloudIndex; // index punktu w oryginalnej chmurze
    int axis;       // wspolrzedna po ktorej tniemy przestrzen x / y / z
    std::unique_ptr<KdTreeNode> left;
    std::unique_ptr<KdTreeNode> right;

    // member initializer list
    KdTreeNode(Point3D pt, int idx, int ax)
        : point(pt), cloudIndex(idx), axis(ax), left(nullptr), right(nullptr)
    {
    }
};

class KdTree
{
    std::unique_ptr<KdTreeNode> root;

    // Median-based construction - zamiast klasycznego insertowania po kolei - mamy posortowane dane
    // gwarantuje idealnie zbalansowane drzewo
    // - left / right - rekurencyjny zakres budowy
    // - depth - os ciecia - x / y / z
    std::unique_ptr<KdTreeNode> buildRecursive(std::vector<std::pair<Point3D, int>> &cloud,
                                               int left, int right, int depth);

  public:
    // member initializer list
    KdTree() : root(nullptr) {}

    void build(const std::vector<Point3D> &cloud);
    std::vector<Point3D> searchKnn(const std::vector<Point3D> &cloud, int k);
};
