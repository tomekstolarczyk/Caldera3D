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
// 2 K-NEAREST NEIGHBOURS SEARCH
// -----------------------------------------------------------------

// helper function
static float distSquared(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

std::vector<int> KdTree::searchKnn(const Point3D &target, int k) const
{
    // 1. vector result initialization and edge cases handling
    std::vector<int> result; // bedziemy zwracali indeksy punktow - stad sam int wystarczy
    if (root == NULL || k <= 0)
    {
        return result;
    }

    // 2. kolejka priorytetowa - bedziemy trzymac tu k najblizszych sasiadow
    // wybralismy ta stukture bo daje ona dostep do max elementu w O(1)
    std::priority_queue<Neighbour> maxHeap;

    // 3. wywolanie rekurencyjne funkcji ktora wypelni nasza kolejke
    searchKnnRecursive(root.get(), target, k, maxHeap);

    // 4. przepakowujemy z kolejki do wektora wynikow
    result.reserve(maxHeap.size());
    while (!maxHeap.empty())
    {
        result.push_back(maxHeap.top().index);
        maxHeap.pop();
    }

    // 5. kolejka oddaje elementy od najdalszego do najblizszego wiec odwracamy wektor result
    std::reverse(result.begin(), result.end());

    return result;
}

void KdTree::searchKnnRecursive(const KdTreeNode *node, const Point3D &target, int k,
                                std::priority_queue<Neighbour> &maxHeap) const
{
    // TODO
}

// -----------------------------------------------------------------
// 3 RADIUS SEARCH
// -----------------------------------------------------------------