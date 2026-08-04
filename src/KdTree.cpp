#include "KdTree.hpp"
#include <algorithm>

// -----------------------------------------------------------------
// 1 TREE BUILDING
// -----------------------------------------------------------------

void KdTree::build(const std::vector<Point3D> &cloud)
{

    if (cloud.empty())
        return;

    // 1 pakujemy chmure do wektora par punkt-indeks
    std::vector<std::pair<Point3D, int>> cloud_paired;
    cloud_paired.reserve(cloud.size());

    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloud_paired.push_back({cloud[i], static_cast<int>(i)});
    }

    // 2 przekazujemy calosc dalej do faktycznej funkcji budujacej
    // - left/right caly zakres & depth startujacy od 0 (pierwsza wspolrzedna - x)
    root = buildRecursive(cloud_paired, 0, static_cast<int>(cloud.size() - 1), 0);
}

std::unique_ptr<KdTreeNode> KdTree::buildRecursive(std::vector<std::pair<Point3D, int>> &cloud,
                                                   int left, int right, int depth)
{
    // 1 warunek stopu rekurencji
    if (left > right)
    {
        return nullptr;
    }

    // 2 wybieramy os ciecia
    int axis = depth % 3; // mod 3 (x,y,z)

    // 3 mediana i przemeblowanie - wsyzstko co mniejsze na lewo, wszystko co wieksze na prawo
    // (std::nth_element)
    // int mid = (left + right) / 2;
    int mid = left + ((right - left) / 2); // more stable numerically - smaller overflow risk
    std::nth_element(cloud.begin() + left, cloud.begin() + mid, cloud.begin() + right + 1,
                     // lambda function for element comparison
                     [axis](const std::pair<Point3D, int> &a, const std::pair<Point3D, int> &b)
                     {
                         if (axis == 0) // x
                         {
                             return a.first.x < b.first.x;
                         }
                         else if (axis == 1) // y
                         {
                             return a.first.y < b.first.y;
                         }
                         return a.first.z < b.first.z; // z
                     });

    // 4 doklejamy wezel do drzewa
    std::unique_ptr<KdTreeNode> node =
        std::make_unique<KdTreeNode>(cloud[mid].first, cloud[mid].second, axis);

    // 5 kontynuujemy rekurencje
    node->left = buildRecursive(cloud, left, mid - 1, depth + 1);
    node->right = buildRecursive(cloud, mid + 1, right, depth + 1);

    return node;
}

// -----------------------------------------------------------------
// 2 NEAREST NEIGHBOURS
// -----------------------------------------------------------------

// todo