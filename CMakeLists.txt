cmake_minimum_required(VERSION 3.14)
project(Caldera3D VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# 1. download pybind
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# 2. srces
set(CALDERA_SOURCES 
    src/PointCloud.cpp 
    src/VoxelGrid.cpp 
    src/KdTree.cpp 
    src/StatisticalOutlierRemoval.cpp
    src/PassThroughFilter.cpp
    src/Ransac.cpp
    src/EuclideanClustering.cpp
    src/SpatialFilter.cpp
    src/PCA.cpp
    src/TargetMatching.cpp
    src/GraspPoseEstimation.cpp
)

# 3. .EXE 
add_executable(caldera src/main.cpp ${CALDERA_SOURCES})
target_include_directories(caldera PRIVATE ${CMAKE_SOURCE_DIR}/external)

# 4. .pyd / .so
pybind11_add_module(caldera_engine src/bindings.cpp ${CALDERA_SOURCES})
target_include_directories(caldera_engine PRIVATE ${CMAKE_SOURCE_DIR}/external)