# Caldera3D 🌋

An ultra-fast 3D point cloud processor designed for autonomous manipulator bin-picking tasks. Written in C++ with Python bindings.

## Core Features & Processing Pipeline

Caldera3D processes point clouds through a highly optimized pipeline, designed step-by-step for industrial reliability:
*   **Data Ingestion & Setup:** Fast 3D Point Cloud parsing (`.ply`) with efficient internal data structure handling.
*   **Workspace Cropping:** Pass-Through Filtering to immediately discard out-of-bounds data.
*   **Spatial Acceleration:** Custom Kd-Tree data structure enabling fast Radius and K-NN searches.
*   **Downsampling & Noise Reduction:** Density reduction via Voxel Grid filtering, followed by high-performance Statistical Outlier Removal (SOR) to eliminate floating artifacts.
*   **Advanced Scene Parsing:** Multi-Plane RANSAC segmentation. Identifies and removes large environmental planes (floors, walls) and extracts the exact mathematical equation of the work table surface.
*   **Object Extraction:** Euclidean Cluster Extraction to segment the remaining point cloud into distinct, isolated items.
*   **Spatial Heuristics:** Centroid-based spatial filtering - ensures only objects resting above the detected table plane are processed.
*   **Dimensional Analysis & Target Matching:** Principal Component Analysis (PCA) is used to compute accurate Oriented Bounding Boxes (OBB). The engine calculates exact dimensions (L/W/H in mm) to find and match specific items requested by a factory PLC (e.g., "find the 20x15 cm package").
*   **Robotic Integration (Grasp Pose Estimation):** Intelligent 6-DoF grasp calculation that determines the safest approach angle to avoid table/object collisions, outputting Universal TCP coordinates and stroke limits for parallel jaw grippers.
*   **Debugging Tools:** Includes complementary Python scripts utilizing Open3D for visual verification of the C++ pipeline outputs.

## Performance Showcase

Below is a sample execution log on a ~460k point cloud:

```text
===================
     Caldera3D     
===================
[SUCCESS] Dane w pamieci! Letsgo
Liczba punktow PRZED (RAW): 460400

[INFO] Nakladam filtr Pass-Through...
Liczba punktow PO Pass-Through: 344509

[INFO] Voxelizujemy chmure (Voxel Size: 0.01m)...
Liczba punktow PO VoxelGrid:    25864

Pierwsze 5 punktow nowej chmury:
Punkt [0]: X=-0.113863, Y=0.227323, Z=-1.3807
Punkt [1]: X=-0.67159, Y=0.166595, Z=-1.27105
Punkt [2]: X=-0.79602, Y=0.070836, Z=-1.1762
Punkt [3]: X=0.764604, Y=0.00235525, Z=-1.44613
Punkt [4]: X=0.0756248, Y=-0.61063, Z=-1.32892

[INFO] Zapisuje zdownsamplowana chmure na dysk...
[SUCCESS] Nowy plik zapisany!

[INFO] Rozpoczynam budowe KD-Tree dla 25864 punktow...
[SUCCESS] Drzewo KD-Tree zbudowane pomyslnie!
[CZAS] Budowa drzewa zajela: 4.9781 ms

[INFO] Szukam K=5 najblizszych sasiadow...
[SUCCESS] Znaleziono 5 sasiadow w czasie 0.0119 ms!

[INFO] Szukam punktow w promieniu R=0.1m wokol celu...
[CZAS] KD-Tree Radius Search: 0.0429 ms
[INFO] Liczba znalezionych punktow: 446
[TEST PASSED] Weryfikacja 100% sukcesu: KD-Tree zwrocilo identyczne punkty co Brute-Force!

[INFO] Uruchamiam filtr SOR (k=50, alpha=2)...
[CZAS] Filtr SOR zajal: 178.534 ms
[INFO] Liczba inlierow: 25174 (usunieto 690 punktow szumu!)
[INFO] Zapisuje odszumiona chmure na dysk...
[SUCCESS] Plik table_scene_lms400_sor_filtered.ply zapisany pomyslnie!

[INFO] Uruchamiam RANSAC Bulk Scene Segmentation...
Znalazlem pozioma plaszczyzne z Z = -1.33848
Znalazlem pozioma plaszczyzne z Z = -1.25851
[INFO] Segmentacja zakonczona!
[INFO] Liczba znalezionych plaszyzn: 2
[INFO] Rownanie blatu stolu: 0.00184727x + 0.236418y + 0.14551z + 0.146219 = 0
[INFO] Liczba punktow pozostalych obiektow: 5888
[SUCCESS] Zapisano obiekty do klastrowania!

[INFO] Rozpoczynam klastrowanie...
[INFO] Znaleziono klastrow: 3
 - Zapisano klaster 0 (punktow: 756)
 - Zapisano klaster 1 (punktow: 295)
 - Zapisano klaster 2 (punktow: 4770)

[INFO] Odrzucam klastry ktore nie leza nad stolem...
[INFO] Znaleziono klastrow na stole: 1
 - Zapisano klaster nad stolem 0 (punktow: 295)

[INFO] Finding OOB...
Centroid:
-0.164922, 0.304489, -1.31381
Extents (L, W, H):
0.157255, 0.0825513, 0.0606067
Rotation Matrix:
 0.0761571  -0.996338 -0.0388649
 -0.471906 -0.0703525   0.878837
  0.878353  0.0485891   0.475536

[TARGET MATCHER] Looking for  a 15 cm x 8 cm x 6 cm package: 
[TARGET FOUND] Dopasowano klaster o indexie: 0

=========================================
[FINAL OUTPUT] GRASP POSE (6-DoF):
Pozycja TCP (Tool Center Point - X, Y, Z):
-0.167201, 0.356013, -1.28594
Macierz orientacji chwytaka (Z w gore):
 0.0761571   0.996338 -0.0388649
 -0.471906  0.0703526   0.878837
  0.878353 -0.0485891   0.475536
Krotszy bok paczki - docelowy rozstaw szczek (Package Width):  60.6067 mm
Dluzszy bok chwytanej paczki (Package Length): 82.5513 mm
=========================================
```

## License
This project is licensed under the terms of the MIT license. See the [LICENSE](LICENSE) file for details.