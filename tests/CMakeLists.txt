include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(caldera_tests
    test_kdtree.cpp
    test_voxelgrid.cpp
    test_filters.cpp
    test_pca.cpp
    test_ransac.cpp
)
target_link_libraries(caldera_tests PRIVATE caldera_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(caldera_tests)
