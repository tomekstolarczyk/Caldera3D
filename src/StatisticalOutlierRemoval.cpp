#include "StatisticalOutlierRemoval.hpp"
#include <cmath>

MeanStdDev computeMeanStdDev(const std::vector<float> &values)
{
    if (values.empty())
    {
        return {0.0, 0.0};
    }

    double sum = 0.0;
    for (float v : values)
    {
        sum += v;
    }
    const double mean = sum / static_cast<double>(values.size());

    if (values.size() < 2)
    {
        return {mean, 0.0};
    }

    // two-pass: sum of squared deviations from the mean
    // the one-pass E[x^2] - E[x]^2 formula in float cancels catastrophically when
    // stddev << mean (very uniform point spacing, e.g. after voxelization): at mean/stddev ~ 1000
    // it returns a negative variance, which used to collapse the threshold to the mean
    double sqDevSum = 0.0;
    for (float v : values)
    {
        const double dev = static_cast<double>(v) - mean;
        sqDevSum += dev * dev;
    }
    return {mean, std::sqrt(sqDevSum / static_cast<double>(values.size() - 1))};
}

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
    }

    // 3-5. srednia globalna i odchylenie standardowe
    const MeanStdDev stats = computeMeanStdDev(mean_dists);

    // 6. prog odcinajacy
    float threshold = static_cast<float>(stats.mean + alpha * stats.stddev);

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