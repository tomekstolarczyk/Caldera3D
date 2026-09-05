#include "SpatialFilter.hpp"
#include <cmath>

std::vector<std::vector<Point3D>>
everythingAboveTheTable(const std::vector<std::vector<Point3D>> &dataClusters, float ATable,
                        float BTable, float CTable, float DTable)
{
    std::vector<std::vector<Point3D>> clustersFiltered;

    // na start normalizujemy rownanie plaszyzny
    float denom = std::sqrt(ATable * ATable + BTable * BTable + CTable * CTable);
    float nA = ATable / denom;
    float nB = BTable / denom;
    float nC = CTable / denom;
    float nD = DTable / denom;
    // po normalizacji wynik rownania plaszyzny po wstawieniu do niej punktu to odleglosc tego
    // puntku od niej wyrazona w metrach - najkrótsza, prostopadła odległość Euklidesowa od tego
    // punktu do płaszczyzny

    for (const auto &cluster : dataClusters)
    {

        // 1 liczymy centroid klastra
        float avgX = 0.0f, avgY = 0.0f, avgZ = 0.0f;
        for (int i = 0; i < cluster.size(); i++)
        {
            avgX += cluster[i].x;
            avgY += cluster[i].y;
            avgZ += cluster[i].z;
        }
        avgX /= cluster.size();
        avgY /= cluster.size();
        avgZ /= cluster.size();

        // 2 podstawiamy do rownania plaszyzny stolu
        float odleglosc = nA * avgX + nB * avgY + nC * avgZ + nD;

        // test pod czy nad stolem
        // wiemy juz ze punkt znajduje sie o dana odleglosc od plaszyzny ale nie wiemy czy pod czy
        // nad przemnazamy przez C
        if (odleglosc * nC > -0.01f) // -0.01f zamiast 0.0f jako zabezpiecznie na szum
        {
            clustersFiltered.push_back(cluster);
        }
    }
    // 4
    return clustersFiltered;
}
