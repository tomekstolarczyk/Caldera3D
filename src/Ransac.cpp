#include "Ransac.hpp"
#include "Math.hpp"
#include <random>

std::vector<Point3D> findPlanes(const std::vector<Point3D> &cloud, int k, float threshold)
{
    // okej a wiec  PRZEPIS:
    // x 1 losujemy 3 punkty z chmury
    // - 2 tworzymy rownanie plaszczyzny
    // - 3 dla kazdego punktu sprawdzamy czy lezy na plaszyznie niej wedlug threshold
    // - 4 liczymy ile jest inlinerow
    // x 5 powtarzamy proces k razy
    // - 6 wybieramy najlepsza
    // - 7 update : adaptacyjna liczba iteracji

    // =============================================================================

    // rng
    std::mt19937 mt{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, cloud.size() - 1);

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

        // 2 tworzymy rownanie plaszczyzny
        Point3D vec1 = Math3D::subVectors(planePoints[1], planePoints[0]);
        Point3D vec2 = Math3D::subVectors(planePoints[2], planePoints[0]);
        Point3D cross = Math3D::crossProduct(vec1, vec2);
        // tutaj znajdziemy A,B,C i D zaraz i mamy plaszczyzne

        // 3 threshold - odleglosc od plasczyzny
        // 4 zliczamy inlinerow
        int count = 0;
        for (int i = 0; i < cloud.size(); i++)
        {
            // TODO
        }
    }
}