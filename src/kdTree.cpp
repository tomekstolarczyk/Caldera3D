#include "kdTree.hpp"
#include <algorithm>

void build(const std::vector<Point3D> &cloud)
{

    if (cloud.empty())
        return;

    // 1 pakujemy chmure do wektora par punkt-indeks
    std::vector<std::pair<Point3D, int>> cloud_paired;
    cloud_paired.reserve(cloud.size());

    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloud_paired.push_back({cloud[i], i});
    }

    // 2 przekazujemy calosc dalej do faktycznej funkcji budujacej
    // - left/right caly zakres & depth startujacy od 0 (pierwsza wspolrzedna - x)
    buildRecursive(cloud_paired, 0, cloud.size() - 1, 0);
}

std::unique_ptr<KdTreeNode> buildRecursive(std::vector<std::pair<Point3D, int>> &cloud, int left,
                                           int right, int depth)
{
}
