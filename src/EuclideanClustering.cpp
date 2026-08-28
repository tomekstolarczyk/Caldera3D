#include "EuclideanClustering.hpp"
#include <iostream>
#include <queue>

std::vector<std::vector<Point3D>> euclideanClusterization(const std::vector<Point3D> &pointCloud,
                                                          const KdTree &kdtree,
                                                          float maxPointDistance)
{
    std::vector<std::vector<Point3D>> result;
    // 1 tworzymy maske logiczna punktow processed
    std::vector<bool> processed(pointCloud.size(), false);

    // 2 idziemy przez kazdy punkt po kolei i klastrujemy
    for (int i = 0; i < pointCloud.size(); i++)
    {
        // jesli punkt juz processed to znaczy ze przypisany do konkretnego klastra - pomijamy
        if (processed[i] == true)
        {
            continue;
        }

        // rozpoczynamy budowe klastra
        else
        {
            // tworzymy nowy klaster
            std::vector<Point3D> klaster;
            // tworzymy kolejke dorzucamy tam nasz punkt
            std::queue<Point3D> koleja;
            koleja.push(pointCloud[i]);
            processed[i] = true;
            while (!koleja.empty())
            {
                // znajdujemy sasiadow badanego punktu i dorzucamy do klastra
                Point3D point = koleja.front();
                koleja.pop();
                klaster.push_back(point);
                std::vector<int> cloudNeighbourIndicies =
                    kdtree.searchRadius(point, maxPointDistance);
                // dorzucamy ich kolejno do klastra o ile nie sa przeprocesowani juz
                for (int j = 0; j < cloudNeighbourIndicies.size(); j++)
                {
                    // jesli juz proccessed to continue
                    if (processed[cloudNeighbourIndicies[j]] == true)
                    {
                        continue;
                    }
                    // dorzucamy punkt do klastra i zmieniamy mu flage na processed
                    else
                    {
                        koleja.push(pointCloud[cloudNeighbourIndicies[j]]);
                        processed[cloudNeighbourIndicies[j]] = true;
                    }
                }
            }

            if (klaster.size() < 5)
            {
                continue;
            }
            else
            {
                result.push_back(klaster);
            }
        }
    }
    std::cout << "Znalazlem klastrow: " << result.size() << std::endl;
    return result;
}