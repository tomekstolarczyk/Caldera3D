# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## [0.2.0]

### Added
- `caldera_core` static library target; the demo executable links against it.
- GoogleTest unit tests: k-d tree (KNN and radius search against brute force), voxel grid,
  pass-through, statistical outlier removal, PCA bounding boxes, RANSAC.
- `scripts/download_data.py`: downloads the PCL table scene and converts it to `.ply`
  (includes a NumPy-only LZF decoder for `binary_compressed` `.pcd` files).
- GitHub Actions CI: build, unit tests and an end-to-end run of the demo.
- The demo takes the input path as its first argument.

### Changed
- Voxel grid hashes voxels by a packed `uint64` key instead of a formatted `std::string`.
- Clustering runs on the RANSAC output in memory instead of re-reading it from disk.

### Fixed
- Statistical outlier removal: standard deviation computed two-pass in double. The one-pass
  float formula returned a negative variance when the neighbour distances were very uniform
  (mean / stddev around 1000), which collapsed the threshold to the mean.

### Removed
- `scripts/convert_PCD_to_PLY.py` (hard-coded Windows paths, Open3D dependency).

## [0.1.0]

### Added
- Initial C++ repository architecture and CMake setup.
- Basic `Point3D` structure and `PointCloud` container class.
- Fast ASCII `.ply` file parser.
- Perception pipeline: pass-through, voxel grid, k-d tree, SOR, multi-plane RANSAC,
  Euclidean clustering, above-table filter, PCA bounding boxes, target matching, grasp pose.