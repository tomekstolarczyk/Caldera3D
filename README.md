# Caldera3D 🌋

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.14+-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
[![Eigen](https://img.shields.io/badge/Eigen-3.4-D12424)](https://eigen.tuxfamily.org/)

**An ultra-fast 3D point cloud processor designed for autonomous manipulator bin-picking tasks. Written in C++ with Python bindings.**

<p align="center">
  <img src="caldera3d_dynamic.gif" width="80%" style="border-radius: 6px;" alt="Inverse Kinematics Demo" />
</p>

## Core Features & Processing Pipeline

Caldera3D processes point clouds through a highly optimized pipeline, designed step-by-step for industrial reliability:
1) **Data Ingestion & Setup:** Fast 3D Point Cloud parsing (`.ply`) with efficient internal data structure handling.
2) **Workspace Cropping:** Pass-Through Filtering to immediately discard out-of-bounds data.
3) **Spatial Acceleration:** Custom Kd-Tree data structure enabling fast Radius and K-NN searches.
4) **Downsampling & Noise Reduction:** Density reduction via Voxel Grid filtering, followed by high-performance Statistical Outlier Removal (SOR) to eliminate floating artifacts.
5) **Advanced Scene Parsing:** Multi-Plane RANSAC segmentation. Identifies and removes large environmental planes (floors, walls) and extracts the exact mathematical equation of the work table surface.
6) **Object Extraction:** Euclidean Cluster Extraction to segment the remaining point cloud into distinct, isolated items.
7) **Spatial Heuristics:** Centroid-based spatial filtering - ensures only objects resting above the detected table plane are processed.
8) **Dimensional Analysis & Target Matching:** Principal Component Analysis (PCA) is used to compute accurate Oriented Bounding Boxes (OBB). The engine calculates exact dimensions (L/W/H in mm) to find and match specific items requested by a factory PLC (e.g., "find the 20x15 cm package").
9) **Robotic Integration (Grasp Pose Estimation):** Intelligent 6-DoF grasp calculation that determines the safest approach angle to avoid table/object collisions, outputting Universal TCP coordinates and stroke limits for parallel jaw grippers.
10) **Debugging Tools:** Includes complementary Python scripts utilizing Open3D for visual verification of the C++ pipeline outputs.
<br><br>
<p align="center">
  <img src="caldera3d.png" width="95%" style="border-radius: 6px;" alt="Inverse Kinematics Demo" />
</p>

## License
This project is licensed under the terms of the MIT license. See the [LICENSE](LICENSE) file for details.
