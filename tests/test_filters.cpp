#include "KdTree.hpp"
#include "PassThroughFilter.hpp"
#include "StatisticalOutlierRemoval.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <gtest/gtest.h>

TEST(PassThrough, KeepsOnlyPointsInsideBox)
{
    std::vector<Point3D> cloud{{0, 0, 0}, {1, 1, 1}, {2, 0, 0}, {0.5f, -0.1f, 0.5f}};
    BoundingBox box{0, 1, 0, 1, 0, 1};
    auto out = passThroughFilter(box, cloud);
    ASSERT_EQ(out.size(), 2u); // bounds are inclusive
    EXPECT_EQ(out[0].x, 0.0f);
    EXPECT_EQ(out[1].x, 1.0f);
}

TEST(SOR, RemovesIsolatedOutliers)
{
    // dense 1 m cube plus a few far away points
    auto cloud = randomCloud(2000, 0.0f, 1.0f);
    const size_t inlierCount = cloud.size();
    cloud.push_back({10, 10, 10});
    cloud.push_back({-10, 5, 0});
    cloud.push_back({0, -8, 3});

    KdTree tree;
    tree.build(cloud);
    auto kept = filterSOR(cloud, tree, 20, 1.0f);

    for (size_t i = inlierCount; i < cloud.size(); i++)
    {
        EXPECT_EQ(std::count(kept.begin(), kept.end(), static_cast<int>(i)), 0)
            << "outlier " << i << " was kept";
    }
    // the vast majority of the cube survives
    EXPECT_GT(kept.size(), inlierCount * 9 / 10);
}

TEST(SOR, TooFewPointsKeepsEverything)
{
    std::vector<Point3D> cloud{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    KdTree tree;
    tree.build(cloud);
    EXPECT_EQ(filterSOR(cloud, tree, 10, 1.0f).size(), cloud.size());
}

TEST(SOR, MeanStdDevEdgeCases)
{
    EXPECT_EQ(computeMeanStdDev({}).stddev, 0.0);
    EXPECT_EQ(computeMeanStdDev({3.0f}).mean, 3.0);
    EXPECT_EQ(computeMeanStdDev({3.0f}).stddev, 0.0);
    EXPECT_EQ(computeMeanStdDev({2.0f, 2.0f, 2.0f}).stddev, 0.0);
}

TEST(SOR, MeanStdDevAccurateWhenSpreadIsTiny)
{
    // regression: mean / stddev = 1000, like neighbour distances in a voxelized cloud
    // the old one-pass float formula returned a *negative* variance here
    std::mt19937 rng(3);
    std::normal_distribution<float> dist(0.01f, 1e-5f);
    std::vector<float> values(50000);
    for (auto &v : values)
    {
        v = dist(rng);
    }

    MeanStdDev s = computeMeanStdDev(values);
    EXPECT_NEAR(s.mean, 0.01, 1e-6);
    EXPECT_NEAR(s.stddev, 1e-5, 2e-7); // sampling error of the stddev is ~0.3% here
}
