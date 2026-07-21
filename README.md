# Caldera3D 🌋

**Caldera3D** is an ultra-fast 3D Point Cloud Processing engine written in modern C++17.

## Features (Planned)
- Fast ASCII/Binary 3D Point Cloud Parser (.ply, .pcd)
- Spatial acceleration structures (Kd-Tree, Voxel Grid filtering)
- RANSAC plane segmentation for ground removal
- Euclidean Cluster Extraction
- Multithreading optimization (OpenMP / C++ Threads)
- Python bindings via Pybind11

## Building and Running

### Prerequisites
- CMake >= 3.10
- C++17 compliant compiler (GCC, Clang, MSVC)

### Build Instructions
```bash
mkdir build && cd build
cmake ..
make
./caldera