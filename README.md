# Project 3: Two-View Camera Motion Estimation

A C++ project for learning feature matching, two-view geometry,
camera motion estimation, and visual odometry.

## Current status

- [x] Minimal CMake project
- [x] OpenCV integration
- [x] Image loading
- [x] Two-frame ORB feature extraction and visualization
- [x] Feature matching
- [x] Raw brute-force descriptor matching
- [x] Cross-check matching
- [ ] KNN ratio-test matching
- [ ] Geometric verification
- [ ] Essential matrix estimation
- [ ] Relative pose recovery

## Requirements

- Visual Studio 2026
- CMake
- C++17

## Run

Pass two consecutive image frames to the executable:

```powershell
out/build/local-debug/project_3_vo.exe data/img1.jpg data/img2.jpg
```

