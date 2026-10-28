#include "VoxelGrid.hpp"
#include "test_helpers.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <stdexcept>

TEST(VoxelGrid, EmptyInput)
{
    EXPECT_TRUE(voxelizePointCloud({}, 0.01f).empty());
}

TEST(VoxelGrid, RejectsNonPositiveVoxelSize)
{
    std::vector<Point3D> cloud{{0, 0, 0}};
    EXPECT_THROW(voxelizePointCloud(cloud, 0.0f), std::invalid_argument);
    EXPECT_THROW(voxelizePointCloud(cloud, -1.0f), std::invalid_argument);
}

TEST(VoxelGrid, PointsInOneVoxelAreAveraged)
{
    std::vector<Point3D> cloud{{0.0f, 0.0f, 0.0f}, {0.4f, 0.2f, 0.0f}, {0.2f, 0.4f, 0.6f}};
    auto out = voxelizePointCloud(cloud, 1.0f);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].x, 0.2f, 1e-6f);
    EXPECT_NEAR(out[0].y, 0.2f, 1e-6f);
    EXPECT_NEAR(out[0].z, 0.2f, 1e-6f);
}

TEST(VoxelGrid, OnePointPerOccupiedVoxel)
{
    // 4 x 3 x 2 grid of points, one in the middle of each 1 m voxel, plus a jittered copy
    std::vector<Point3D> cloud;
    for (int x = 0; x < 4; x++)
        for (int y = 0; y < 3; y++)
            for (int z = 0; z < 2; z++)
            {
                cloud.push_back({x + 0.5f, y + 0.5f, z + 0.5f});
                cloud.push_back({x + 0.6f, y + 0.4f, z + 0.5f});
            }
    // the cloud minimum is 0.4 in y, so shift the grid origin: keep a point at the origin
    cloud.push_back({0.0f, 0.0f, 0.0f});

    auto out = voxelizePointCloud(cloud, 1.0f);
    // origin point shares voxel (0,0,0) with the first grid cell
    EXPECT_EQ(out.size(), 4u * 3u * 2u);
}

TEST(VoxelGrid, OutputCountNeverExceedsInput)
{
    auto cloud = randomCloud(10000, -1.0f, 1.0f);
    for (float voxel : {0.01f, 0.1f, 0.5f})
    {
        auto out = voxelizePointCloud(cloud, voxel);
        EXPECT_LE(out.size(), cloud.size());
        EXPECT_GT(out.size(), 0u);
    }
    // voxel larger than the whole cloud -> a single centroid
    EXPECT_EQ(voxelizePointCloud(cloud, 10.0f).size(), 1u);
}

TEST(VoxelGrid, RejectsCloudTooLargeForKey)
{
    // 1 km span at 0.1 mm voxels = 10^7 voxels per axis > 2^21
    std::vector<Point3D> cloud{{0, 0, 0}, {1000.0f, 0, 0}};
    EXPECT_THROW(voxelizePointCloud(cloud, 0.0001f), std::invalid_argument);
}
