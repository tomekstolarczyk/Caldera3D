#include "KdTree.hpp"
#include "PassThroughFilter.hpp"
#include "PointCloud.hpp"
#include "Ransac.hpp"
#include "StatisticalOutlierRemoval.hpp"
#include "VoxelGrid.hpp"
#include <algorithm> // std::min, std::sort
#include <chrono>    // pomiar czasu
#include <iostream>
#include <vector>

// just for testing
static std::vector<int> bruteForceRadius(const std::vector<Point3D> &cloud, const Point3D &target,
                                         float radius)
{
    std::vector<int> indices;
    float r_sq = radius * radius;

    for (size_t i = 0; i < cloud.size(); ++i)
    {
        float dx = cloud[i].x - target.x;
        float dy = cloud[i].y - target.y;
        float dz = cloud[i].z - target.z;
        if (dx * dx + dy * dy + dz * dz <= r_sq)
        {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

int main()
{
    std::cout << "===================" << std::endl;
    std::cout << "     Caldera3D     " << std::endl;
    std::cout << "===================" << std::endl;

    PointCloud rawCloud;

    // 1. Wczytujemy cala surowa chmure do pamieci
    if (!rawCloud.loadData("data/table_scene_lms400.ply"))
    {
        std::cout << "[FATAL] Nie udalo sie wczytac pliku :(" << std::endl;
        return -1;
    }

    std::cout << "[SUCCESS] Dane w pamieci! Letsgo" << std::endl;
    std::cout << "Liczba punktow PRZED (RAW): " << rawCloud.size() << std::endl;

    // 2. Pass-Through Filter - Wycinamy obszar roboczy
    // useless rn but works
    BoundingBox roi;
    roi.minX = -1.0f;
    roi.maxX = 1.0f;
    roi.minY = -1.0f;
    roi.maxY = 0.5f;
    roi.minZ = -1.5f;
    roi.maxZ = -0.5f;

    std::cout << "\n[INFO] Nakladam filtr Pass-Through..." << std::endl;
    std::vector<Point3D> croppedPoints = passThroughFilter(roi, rawCloud.getPoints());
    PointCloud croppedCloud;
    croppedCloud.setPoints(croppedPoints);
    std::cout << "Liczba punktow PO Pass-Through: " << croppedCloud.size() << std::endl;

    // 3. Downsampling - Voxel Grid Filter (0.01m = 1cm)
    std::cout << "\n[INFO] Voxelizujemy chmure (Voxel Size: 0.01m)..." << std::endl;
    std::vector<Point3D> downsampledVector = voxelizePointCloud(croppedCloud.getPoints(), 0.01f);
    PointCloud filteredCloud;
    filteredCloud.setPoints(downsampledVector);

    std::cout << "Liczba punktow PO VoxelGrid:    " << filteredCloud.size() << std::endl;

    // 4. Printujemy pierwsze 5 punktow przefiltrowanej chmury
    int pointsToPrint = std::min(5, static_cast<int>(filteredCloud.size()));
    std::cout << "\nPierwsze " << pointsToPrint << " punktow nowej chmury:" << std::endl;
    const std::vector<Point3D> &filteredPts = filteredCloud.getPoints();
    for (int i = 0; i < pointsToPrint; i++)
    {
        std::cout << "Punkt [" << i << "]: X=" << filteredPts[i].x << ", Y=" << filteredPts[i].y
                  << ", Z=" << filteredPts[i].z << std::endl;
    }

    // 5. Zapisujemy do nowego pliku dla podgladu
    std::cout << "\n[INFO] Zapisuje zdownsamplowana chmure na dysk..." << std::endl;
    if (filteredCloud.saveData("data/table_scene_lms400_voxel_downsampled.ply"))
    {
        std::cout << "[SUCCESS] Nowy plik zapisany!" << std::endl;
    }

    // 6. Budujemy z chmury punktow KD-Tree
    std::cout << "\n[INFO] Rozpoczynam budowe KD-Tree dla " << filteredCloud.size() << " punktow..."
              << std::endl;
    KdTree tree;
    auto start_build = std::chrono::high_resolution_clock::now();
    tree.build(filteredCloud.getPoints());
    auto end_build = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> build_ms = end_build - start_build;
    std::cout << "[SUCCESS] Drzewo KD-Tree zbudowane pomyslnie!" << std::endl;
    std::cout << "[CZAS] Budowa drzewa zajela: " << build_ms.count() << " ms\n" << std::endl;

    // 7. Test KNN: szukamy K=5 najbliższych sąsiadów
    if (!filteredPts.empty())
    {
        Point3D target = filteredPts[0];
        int k = 5;

        std::cout << "[INFO] Szukam K=" << k << " najblizszych sasiadow..." << std::endl;

        auto start_knn = std::chrono::high_resolution_clock::now();
        std::vector<int> neighborIndices = tree.searchKnn(target, k);
        auto end_knn = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> knn_ms = end_knn - start_knn;
        std::cout << "[SUCCESS] Znaleziono " << neighborIndices.size() << " sasiadow w czasie "
                  << knn_ms.count() << " ms!" << std::endl;
    }

    // 8. TEST RADIUS SEARCH
    if (!filteredPts.empty())
    {
        Point3D target = filteredPts[0];
        float radius = 0.10f; // szukamy w promieniu 10 cm
        std::cout << "\n[INFO] Szukam punktow w promieniu R=" << radius << "m wokol celu..."
                  << std::endl;
        // 1 Wykonanie zapytania przez KD-Tree
        auto start_radius = std::chrono::high_resolution_clock::now();
        std::vector<int> kd_results = tree.searchRadius(target, radius);
        auto end_radius = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> radius_ms = end_radius - start_radius;
        // 2 Wykonanie zapytania referencyjnego (Brute-Force)
        std::vector<int> brute_results = bruteForceRadius(filteredPts, target, radius);
        std::cout << "[CZAS] KD-Tree Radius Search: " << radius_ms.count() << " ms" << std::endl;
        std::cout << "[INFO] Liczba znalezionych punktow: " << kd_results.size() << std::endl;
        // 3 Porównanie wyników
        std::sort(kd_results.begin(), kd_results.end());
        std::sort(brute_results.begin(), brute_results.end());
        if (kd_results == brute_results)
        {
            std::cout << "[TEST PASSED] Weryfikacja 100% sukcesu: KD-Tree zwrocilo identyczne "
                         "punkty co Brute-Force!\n"
                      << std::endl;
        }
        else
        {
            std::cout << "[TEST FAILED] BŁĄD LOGIKI: Rozbieznosc pomiedzy KD-Tree a Brute-Force!"
                      << std::endl;
            std::cout << "KD-Tree znalazlo: " << kd_results.size()
                      << " punktow, a Brute-Force: " << brute_results.size() << " punktow."
                      << std::endl;
        }
    }

    // 9. TEST SOR FILTER (Usuwanie szumu)
    std::vector<Point3D> cleanPoints;
    if (!filteredPts.empty())
    {
        int sor_k = 50;
        float sor_alpha = 2.0f;
        std::cout << "[INFO] Uruchamiam filtr SOR (k=" << sor_k << ", alpha=" << sor_alpha << ")..."
                  << std::endl;
        auto start_sor = std::chrono::high_resolution_clock::now();
        std::vector<int> inliers = filterSOR(filteredPts, tree, sor_k, sor_alpha);
        auto end_sor = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> sor_ms = end_sor - start_sor;
        std::cout << "[CZAS] Filtr SOR zajal: " << sor_ms.count() << " ms" << std::endl;
        std::cout << "[INFO] Liczba inlierow: " << inliers.size() << " (usunieto "
                  << (filteredPts.size() - inliers.size()) << " punktow szumu!)" << std::endl;
        // Wyciąganie poprawnych punktów na podstawie zwróconych indeksów
        cleanPoints.reserve(inliers.size());
        for (int idx : inliers)
        {
            cleanPoints.push_back(filteredPts[idx]);
        }
        // Zapis do pliku
        PointCloud cleanCloud;
        cleanCloud.setPoints(cleanPoints);
        std::cout << "[INFO] Zapisuje odszumiona chmure na dysk..." << std::endl;
        if (cleanCloud.saveData("data/table_scene_lms400_sor_filtered.ply"))
        {
            std::cout << "[SUCCESS] Plik table_scene_lms400_sor_filtered.ply zapisany pomyslnie!\n"
                      << std::endl;
        }
    }

    // 10. TEST RANSAC (Wykrywanie płaszczyzny stołu)
    if (!cleanPoints.empty())
    {
        int ransac_iter = 1000;
        float ransac_threshold = 0.015f;

        std::cout << "\n[INFO] Uruchamiam RANSAC (Iteracje: " << ransac_iter
                  << ", Prog: " << ransac_threshold << "m)..." << std::endl;

        auto start_ransac = std::chrono::high_resolution_clock::now();
        // Odbieramy pare wektorow
        auto [tableInliers, objectsOutliers] =
            findPlanes(cleanPoints, ransac_iter, ransac_threshold);
        auto end_ransac = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ransac_ms = end_ransac - start_ransac;

        std::cout << "[CZAS] RANSAC zajal: " << ransac_ms.count() << " ms" << std::endl;
        std::cout << "[INFO] Znaleziono blat stolu: " << tableInliers.size() << " punktow."
                  << std::endl;
        std::cout << "[INFO] Wyizolowano obiekty: " << objectsOutliers.size() << " punktow."
                  << std::endl;

        PointCloud tableCloud;
        tableCloud.setPoints(tableInliers);
        tableCloud.saveData("data/table_scene_lms400_ransac_table.ply");

        PointCloud objectsCloud;
        objectsCloud.setPoints(objectsOutliers);
        objectsCloud.saveData("data/table_scene_lms400_ransac_objects.ply");
    }

    return 0;
}