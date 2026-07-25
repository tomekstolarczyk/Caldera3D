#include "VoxelGrid.hpp"
#include <cmath>
#include <string>
#include <unordered_map>

std::vector<Point3D> voxelizePointCloud(const std::vector<Point3D> &points, float voxelSize)
{
    // points nie jest jednym obiektem chmury punktow tylko
    // juz jest tymi faktynczymi punktami x,y,z -> bezposredni dostep bez gettera

    if (points.empty())
    {
        return {};
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
    std::unordered_map<std::string, voxelBrick> hashMapa;
    int voxelizedX, voxelizedY, voxelizedZ;
    std::string klucz;
    for (size_t i = 0; i < points.size(); i++)
    {
        // szukamy pudelka
        voxelizedX = static_cast<int>(std::floor((points[i].x - minX) / voxelSize));
        voxelizedY = static_cast<int>(std::floor((points[i].y - minY) / voxelSize));
        voxelizedZ = static_cast<int>(std::floor((points[i].z - minZ) / voxelSize));

        // budujemy klucz
        klucz = std::to_string(voxelizedX) + "_" + std::to_string(voxelizedY) + "_" +
                std::to_string(voxelizedZ);

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