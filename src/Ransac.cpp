#include "Ransac.hpp"
#include "Math.hpp"
#include <random>

std::vector<Point3D> findPlanes(const std::vector<Point3D> &cloud, int k, float threshold)
{
    // okej a wiec  PRZEPIS:
    // x 1 losujemy 3 punkty z chmury
    // x 2 tworzymy rownanie plaszczyzny
    // x 3 dla kazdego punktu sprawdzamy czy lezy na plaszyznie wedlug threshold i zliczamy
    // inlinerow
    // x 4 powtarzamy proces k razy
    // x 5 wybieramy najlepsza iteracje
    // - 7 update : adaptacyjna liczba iteracji : TODO

    // =============================================================================

    if (cloud.size() < 3)
    {
        return {};
    }

    // rng
    std::mt19937 mt{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, cloud.size() - 1);

    // 5 wybieramy najlepsza
    std::vector<Point3D> inlinersBest;
    int maxCount = 0;

    // 4 powtarzamy proces k razy
    for (int iter = 0; iter < k; iter++)
    {
        // 1 lostujemy 3 punkty z chmury
        int idx1 = dist(mt);
        int idx2 = dist(mt);

        while (idx1 == idx2)
        {
            idx2 = dist(mt);
        }

        int idx3 = dist(mt);
        while (idx3 == idx1 || idx3 == idx2)
        {
            idx3 = dist(mt);
        }

        std::vector<Point3D> planePoints;
        planePoints.push_back(cloud[idx1]);
        planePoints.push_back(cloud[idx2]);
        planePoints.push_back(cloud[idx3]);

        // 2 tworzymy rownanie plaszczyzny Ax + By + Cz + D = 0
        Point3D vec1 = Math3D::pointVecSubstraction(planePoints[1], planePoints[0]);
        Point3D vec2 = Math3D::pointVecSubstraction(planePoints[2], planePoints[0]);
        Point3D cross = Math3D::pointVecCrossProduct(vec1, vec2);
        float A = cross.x;
        float B = cross.y;
        float C = cross.z;
        float D = -(A * planePoints[0].x + B * planePoints[0].y + C * planePoints[0].z);

        // 3 dla kazdego punktu sprawdzamy czy lezy na plaszyznie wedlug threshold i zliczamy
        // inlinerow
        float denom = std::sqrt(A * A + B * B + C * C);

        // jesli wylosowalismy 3 punkty w jednej linii to ich iloczyn wektorowy bedzie zerowy
        // takie punkty i tak sa nam do niczego bo nie zbudujemy na nich plaszczyzny wiec wywalamy
        // ta iteracje do smieci straight up
        if (denom < 1e-6f)
        {
            continue;
        }

        std::vector<Point3D> inliners;
        inliners.reserve(cloud.size());
        for (int i = 0; i < cloud.size(); i++)
        {

            float nom = std::abs(A * cloud[i].x + B * cloud[i].y + C * cloud[i].z + D);
            float d = nom / denom;

            if (d <= threshold)
            {
                inliners.push_back(cloud[i]);
            }
        }

        if (inliners.size() > maxCount)
        {
            maxCount = static_cast<int>(inliners.size());
            inlinersBest = std::move(inliners);
        }
    }

    return inlinersBest;
}