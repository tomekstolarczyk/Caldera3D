#include "Ransac.hpp"
#include "test_helpers.hpp"
#include <cmath>
#include <gtest/gtest.h>

TEST(Ransac, FindsNoisyPlane)
{
    // plane z = 0.2 with +-2 mm noise, plus 20% random clutter above it
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> xy(-1.0f, 1.0f), noise(-0.002f, 0.002f);
    std::vector<Point3D> cloud;
    for (int i = 0; i < 4000; i++)
    {
        cloud.push_back({xy(rng), xy(rng), 0.2f + noise(rng)});
    }
    for (const auto &p : randomCloud(1000, 0.3f, 1.0f))
    {
        cloud.push_back(p);
    }

    auto res = ransacFindPlane(cloud, 200, 0.01f);
    float n = std::sqrt(res.A * res.A + res.B * res.B + res.C * res.C);
    ASSERT_GT(n, 0.0f);

    EXPECT_NEAR(std::abs(res.C / n), 1.0f, 1e-2f); // normal along Z
    EXPECT_NEAR(std::abs(res.D / res.C), 0.2f, 5e-3f);
    EXPECT_EQ(res.inliers.size() + res.outliers.size(), cloud.size());
    EXPECT_GE(res.inliers.size(), 4000u);
}

TEST(Ransac, TooFewPoints)
{
    auto res = ransacFindPlane({{0, 0, 0}, {1, 0, 0}}, 100, 0.01f);
    EXPECT_TRUE(res.inliers.empty());
}
