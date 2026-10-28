cmake_minimum_required(VERSION 3.14)
project(Caldera3D VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(CALDERA_BUILD_TESTS "Build unit tests (fetches GoogleTest)" ON)

# core library - the whole perception pipeline, reused by the demo, tests and (later) bindings
add_library(caldera_core STATIC
    src/PointCloud.cpp
    src/PassThroughFilter.cpp
    src/VoxelGrid.cpp
    src/KdTree.cpp
    src/StatisticalOutlierRemoval.cpp
    src/Ransac.cpp
    src/EuclideanClustering.cpp
    src/SpatialFilter.cpp
    src/PCA.cpp
    src/TargetMatching.cpp
    src/GraspPoseEstimation.cpp
)
target_include_directories(caldera_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
# eigen (vendored, header-only)
target_include_directories(caldera_core SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/external)

# demo - full pipeline on a tabletop scene
add_executable(caldera src/main.cpp)
target_link_libraries(caldera PRIVATE caldera_core)

if(CALDERA_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
