#include "VoxelGrid.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

// 21 bits per axis -> three indices packed into one uint64 key
// (2^21 voxels per axis = ~21 km at 1 cm voxels, plenty for a robot workspace)
static constexpr int kBitsPerAxis = 21;
static constexpr std::uint64_t kAxisMask = (std::uint64_t{1} << kBitsPerAxis) - 1;

static std::uint64_t packVoxelKey(std::uint64_t ix, std::uint64_t iy, std::uint64_t iz)
{
    return ix | (iy << kBitsPerAxis) | (iz << (2 * kBitsPerAxis));
}

std::vector<Point3D> voxelizePointCloud(const std::vector<Point3D> &points, float voxelSize)
{
    // points nie jest jednym obiektem chmury punktow tylko
    // juz jest tymi faktynczymi punktami x,y,z -> bezposredni dostep bez gettera

    if (points.empty())
    {
        return {};
    }
    if (!(voxelSize > 0.0f))
    {
        throw std::invalid_argument("voxelizePointCloud: voxelSize must be positive");
    }
    // 1 find min for every coordinate - will be needed in step 2
    float minX = points[0].x, minY = points[0].y, minZ = points[0].z;
    for (size_t i = 1; i < points.size(); i++)
    {
        if (points[i].x < minX)
        {
            minX = points[i].x;
        }
        if (points[i].y < minY)
        {
            minY = points[i].y;
        }
        if (points[i].z < minZ)
        {
            minZ = points[i].z;
        }
    }

    // 2 przyporzodkowujemy kazdy punkt do odpowiedniego woksela (using hash mapa)
    // integer key instead of a std::string one - no allocation and formatting per point
    std::unordered_map<std::uint64_t, voxelBrick> hashMapa;
    hashMapa.reserve(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        // szukamy pudelka - indices are >= 0 because we subtract the minimum
        auto voxelizedX = static_cast<std::uint64_t>(std::floor((points[i].x - minX) / voxelSize));
        auto voxelizedY = static_cast<std::uint64_t>(std::floor((points[i].y - minY) / voxelSize));
        auto voxelizedZ = static_cast<std::uint64_t>(std::floor((points[i].z - minZ) / voxelSize));

        if (voxelizedX > kAxisMask || voxelizedY > kAxisMask || voxelizedZ > kAxisMask)
        {
            throw std::invalid_argument("voxelizePointCloud: cloud too large for this voxelSize");
        }

        // budujemy klucz
        std::uint64_t klucz = packVoxelKey(voxelizedX, voxelizedY, voxelizedZ);

        // wrzucamy do pudelka
        auto &brick = hashMapa[klucz];
        brick.count++;
        brick.sumX += points[i].x;
        brick.sumY += points[i].y;
        brick.sumZ += points[i].z;
    }

    // 3 usredniamy i finito
    std::vector<Point3D> cloudDownsampled;
    cloudDownsampled.reserve(hashMapa.size());
    for (const auto &pair : hashMapa)
    {
        // pair.first - klucz
        // pair.second - voxelBrick

        float newX = pair.second.sumX / pair.second.count;
        float newY = pair.second.sumY / pair.second.count;
        float newZ = pair.second.sumZ / pair.second.count;

        cloudDownsampled.push_back({newX, newY, newZ});
    }

    return cloudDownsampled;
}