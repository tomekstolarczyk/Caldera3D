#include "KdTree.hpp"
#include "PointCloud.hpp"
#include "VoxelGrid.hpp"
#include <chrono> // for time measures
#include <iostream>

int main()
{
    std::cout << "===================" << std::endl;
    std::cout << "     Caldera3D     " << std::endl;
    std::cout << "===================" << std::endl;

    PointCloud cloud;

    // 1. Wczytujemy cala chmure do pamieci
    if (!cloud.loadData("data/table_scene_lms400.ply"))
    {
        std::cout << "[FATAL] Nie udalo sie wczytac pliku :(" << std::endl;
        return -1;
    }

    std::cout << "[SUCCESS] Dane w pamieci! Letsgo" << std::endl;
    std::cout << "Liczba punktow PRZED: " << cloud.size() << std::endl;

    // 2. Downsampling - Voxel Grid Filter
    std::cout << "\n[INFO] Na start voxelizujemy chmure (Voxel Size: 0.05m)..." << std::endl;
    std::vector<Point3D> downsampledVector = voxelizePointCloud(cloud.getPoints(), 0.05f);
    PointCloud filteredCloud;
    filteredCloud.setPoints(downsampledVector);

    std::cout << "Liczba punktow PO:    " << filteredCloud.size() << std::endl;

    // 3 printujemy pierwsze 5 punktow przefiltrowanej chmury
    int pointsToPrint = std::min(5, (int)filteredCloud.size());
    std::cout << "\nPierwsze " << pointsToPrint << " punktow nowej chmury:" << std::endl;

    const std::vector<Point3D> &filteredPts = filteredCloud.getPoints();
    for (int i = 0; i < pointsToPrint; i++)
    {
        std::cout << "Punkt [" << i << "]: X=" << filteredPts[i].x << ", Y=" << filteredPts[i].y
                  << ", Z=" << filteredPts[i].z << std::endl;
    }

    // 4 zapisujemy do nowego pliku dla podgladu
    std::cout << "\n[INFO] Zapisuje zdownsamplowana chmure na dysk..." << std::endl;
    if (filteredCloud.saveData("data/table_scene_lms400_downsampled.ply"))
    {
        std::cout << "[SUCCESS] Nowy plik zapisany!" << std::endl;
    }

    // 5. budujemy z chmury punktow kdtree
    std::cout << "\n[INFO] Rozpoczynam budowe KD-Tree dla " << filteredCloud.size() << " punktow..."
              << std::endl;

    KdTree tree;
    auto start = std::chrono::high_resolution_clock::now();
    tree.build(filteredCloud.getPoints());
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = end - start;
    std::cout << "[SUCCESS] Drzewo KD-Tree zbudowane pomyslnie! (Brak bledow)" << std::endl;
    std::cout << "[CZAS] Budowa drzewa zajela: " << ms_double.count() << " milisekund!\n"
              << std::endl;

    return 0;
}