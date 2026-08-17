#include "StatisticalOutlierRemoval.hpp"
#include <cmath>

std::vector<int> filterSOR(const std::vector<Point3D> &points, const KdTree &tree, int k,
                           float alpha)
{
    // tree - drzewo kdtree chmury punktow
    // k - liczba sasiadow do ktorych badamy odleglosc
    // alpha - parametr koncowy do odcinania dzwonu gaussa

    // 1. guard clauses
    if (points.empty() || k <= 0 || static_cast<int>(points.size()) <= k)
    {
        std::vector<int> results(points.size());
        for (int i = 0; i < points.size(); i++)
        {
            results[i] = i;
        }
        return results;
    }

    // 2. dla kazdego punktu liczymy jego srednia odleglosc do k sasiadow
    std::vector<float> mean_dists(points.size());
    float dist_sum = 0.0f;    // przyda nam sie do wariancji
    float dist_sq_sum = 0.0f; // przyda nam sie do wariancji
    for (int i = 0; i < points.size(); i++)
    {

        // znadujemy indexy k+1 sasiadow - k+ 1 bo pomijamy samego siebie
        std::vector<int> indexy = tree.searchKnn(points[i], k + 1);
        // teraz dla kazdego liczymy srednia odleglosc
        float dist = 0.0f;
        for (int j = 0; j < indexy.size(); j++)
        {
            if (indexy[j] == i)
            {
                continue;
            }
            // odleglosc euklidesowa
            float dx = (points[indexy[j]].x - points[i].x);
            float dy = (points[indexy[j]].y - points[i].y);
            float dz = (points[indexy[j]].z - points[i].z);
            dist += static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
        }

        dist = dist / (indexy.size() - 1);
        mean_dists[i] = dist;
        dist_sum += dist;
        dist_sq_sum += dist * dist;
    }

    // 3. srednia globalna
    float global_mean = dist_sum / points.size();

    // 4. wariancja
    float var = (dist_sq_sum - (((dist_sum) * (dist_sum)) / points.size())) / (points.size() - 1);
    if (var < 0.0f)
    {
        var = 0.0f;
    }

    // 5. odchylenie standardowe
    float stddev = static_cast<float>(sqrt(var));

    // 6. prog odcinajacy
    float threshold = global_mean + alpha * stddev;

    // 7. wynik
    std::vector<int> results;
    results.reserve(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        if (mean_dists[i] < threshold)
        {
            results.push_back(i);
        }
    }
    return results;
}