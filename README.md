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
- [x] KNN ratio-test matching
- [x] Geometric verification
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

## Matching experiment

Dataset: `data/img1.jpg` and `data/img2.jpg`

| Method | Matches | Retention rate |
|---|---:|---:|
| Raw 1-NN | 1000 | 100.0% |
| Cross-check | 385 | 38.5% |
| KNN ratio test (`r = 0.75`) | 205 | 20.5% |

The displayed top-100 matches showed no obvious visual outliers.
Visual inspection alone does not prove geometric correctness.

| Ratio-test matches | RANSAC inliers | Inlier rate | Threshold |
|---:|---:|---:|---:|
| 205 | 124 | 60.49% | 1.0 px |

### Fundamental matrix diagnostics

- Determinant:
- Singular values:
- Smallest-to-second singular value ratio:

