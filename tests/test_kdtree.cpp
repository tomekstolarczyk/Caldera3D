#include "KdTree.hpp"
#include "Math.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <gtest/gtest.h>

namespace
{

std::vector<int> bruteForceRadius(const std::vector<Point3D> &cloud, const Point3D &q, float r)
{
    std::vector<int> out;
    for (size_t i = 0; i < cloud.size(); i++)
    {
        if (Math3D::distSquared(cloud[i], q) <= r * r)
        {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

std::vector<float> bruteForceKnnDistances(const std::vector<Point3D> &cloud, const Point3D &q,
                                          int k)
{
    std::vector<float> d;
    for (const auto &p : cloud)
    {
        d.push_back(Math3D::distSquared(p, q));
    }
    std::sort(d.begin(), d.end());
    d.resize(std::min<size_t>(k, d.size()));
    return d;
}

} // namespace

TEST(KdTree, EmptyTreeReturnsNothing)
{
    KdTree tree;
    tree.build({});
    EXPECT_TRUE(tree.searchKnn({0, 0, 0}, 5).empty());
    EXPECT_TRUE(tree.searchRadius({0, 0, 0}, 1.0f).empty());
}

TEST(KdTree, SinglePoint)
{
    KdTree tree;
    tree.build({{1, 2, 3}});
    EXPECT_EQ(tree.searchKnn({0, 0, 0}, 3), std::vector<int>{0});
    EXPECT_TRUE(tree.searchRadius({0, 0, 0}, 1.0f).empty());
    EXPECT_EQ(tree.searchRadius({1, 2, 3}, 0.1f), std::vector<int>{0});
}

TEST(KdTree, KnnMatchesBruteForce)
{
    auto cloud = randomCloud(5000, -1.0f, 1.0f);
    KdTree tree;
    tree.build(cloud);

    auto queries = randomCloud(200, -1.2f, 1.2f, 7); // some queries lie outside the cloud
    for (int k : {1, 5, 32})
    {
        for (const auto &q : queries)
        {
            auto idx = tree.searchKnn(q, k);
            ASSERT_EQ(static_cast<int>(idx.size()), k);

            // compare distances, not indices - robust to ties
            std::vector<float> got;
            for (int i : idx)
            {
                got.push_back(Math3D::distSquared(cloud[i], q));
            }
            EXPECT_TRUE(std::is_sorted(got.begin(), got.end())) << "results not nearest-first";
            EXPECT_EQ(got, bruteForceKnnDistances(cloud, q, k));
        }
    }
}

TEST(KdTree, KnnWithKLargerThanCloud)
{
    auto cloud = randomCloud(10, 0.0f, 1.0f);
    KdTree tree;
    tree.build(cloud);
    EXPECT_EQ(tree.searchKnn({0.5f, 0.5f, 0.5f}, 50).size(), cloud.size());
}

TEST(KdTree, RadiusMatchesBruteForce)
{
    auto cloud = randomCloud(5000, -1.0f, 1.0f);
    KdTree tree;
    tree.build(cloud);

    for (const auto &q : randomCloud(200, -1.2f, 1.2f, 7))
    {
        for (float r : {0.05f, 0.2f, 0.5f})
        {
            auto got = tree.searchRadius(q, r);
            std::sort(got.begin(), got.end());
            EXPECT_EQ(got, bruteForceRadius(cloud, q, r));
        }
    }
}

TEST(KdTree, DuplicatePoints)
{
    std::vector<Point3D> cloud(100, Point3D{0.5f, 0.5f, 0.5f});
    KdTree tree;
    tree.build(cloud);
    EXPECT_EQ(tree.searchRadius({0.5f, 0.5f, 0.5f}, 0.01f).size(), cloud.size());
    EXPECT_EQ(tree.searchKnn({0, 0, 0}, 10).size(), 10u);
}
