#pragma once
#include "PointCloud.hpp"
#include <memory>
#include <queue>

// glowna struktura wezla drzewa
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

// stuktura pomocnicza dla kolejki priorytetowej kNN
struct Neighbour
{
    float distance;
    int index;

    // przeciazamy operator porownania - konieczne do kolejki priorytetowej
    bool operator<(const Neighbour &other) const
    {
        return distance < other.distance;
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

    void searchKnnRecursive(const KdTreeNode *node, const Point3D &target, int k,
                            std::priority_queue<Neighbour> &maxHeap) const;

    void searchRadiusRecursive(const KdTreeNode *node, const Point3D &target, float radius,
                               std::vector<int> &results) const;

  public:
    // member initializer list
    KdTree() : root(nullptr) {}

    void build(const std::vector<Point3D> &cloud);

    std::vector<int> searchKnn(const Point3D &target, int k) const;
    std::vector<int> searchRadius(const Point3D &target, float radius) const;
};
