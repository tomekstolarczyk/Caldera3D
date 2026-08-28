cmake_minimum_required(VERSION 3.10)
project(Caldera3D VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include)
add_executable(caldera 
    src/main.cpp 
    src/PointCloud.cpp 
    src/VoxelGrid.cpp 
    src/KdTree.cpp 
    src/StatisticalOutlierRemoval.cpp
    src/PassThroughFilter.cpp
    src/Ransac.cpp
    src/EuclideanClustering.cpp
)