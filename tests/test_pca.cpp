#include "PCA.hpp"
#include "test_helpers.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{

// uniformly filled box with given extents, rotated by `angle` around Z and translated
std::vector<Point3D> boxCloud(float l, float w, float h, float angle, Point3D t)
{
    auto unit = randomCloud(20000, -0.5f, 0.5f);
    // make sure the extremes are hit exactly so the measured extents are exact
    for (float sx : {-0.5f, 0.5f})
        for (float sy : {-0.5f, 0.5f})
            for (float sz : {-0.5f, 0.5f})
                unit.push_back({sx, sy, sz});

    const float c = std::cos(angle), s = std::sin(angle);
    std::vector<Point3D> out;
    for (const auto &p : unit)
    {
        float x = p.x * l, y = p.y * w, z = p.z * h;
        out.push_back({c * x - s * y + t.x, s * x + c * y + t.y, z + t.z});
    }
    return out;
}

} // namespace

TEST(PCA, MeasuresRotatedBox)
{
    auto cloud = boxCloud(0.30f, 0.15f, 0.05f, 0.6f, {1.0f, -2.0f, 0.5f});
    OBB box = findOBB(cloud);

    // a PCA box is not the minimal box: sampling noise tilts the eigenvectors by a few mrad,
    // and a 0.3 m side tilted by ~4 mrad adds ~1 mm to the short extents
    const float tol = 2e-3f;
    EXPECT_NEAR(box.length, 0.30f, tol);
    EXPECT_NEAR(box.width, 0.15f, tol);
    EXPECT_NEAR(box.height, 0.05f, tol);
    EXPECT_NEAR(box.centroid.x, 1.0f, 5e-3f);
    EXPECT_NEAR(box.centroid.y, -2.0f, 5e-3f);
    EXPECT_NEAR(box.centroid.z, 0.5f, 5e-3f);

    // shortest axis (column 0) is the box's Z axis, longest follows the rotation angle
    EXPECT_NEAR(std::abs(box.rotationAxes.col(0).z()), 1.0f, 1e-3f);
    EXPECT_NEAR(std::abs(box.rotationAxes.col(2).x()), std::cos(0.6f), 1e-2f);
}

TEST(PCA, AxesAreOrthonormal)
{
    OBB box = findOBB(boxCloud(0.2f, 0.1f, 0.07f, 1.1f, {0, 0, 0}));
    Eigen::Matrix3f gram = box.rotationAxes.transpose() * box.rotationAxes;
    EXPECT_TRUE(gram.isIdentity(1e-4f)) << gram;
}
