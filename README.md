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
- [x] Camera calibration image validation
- [x] Chessboard corner detection
- [x] Initial camera intrinsic calibration
- [x] Per-view calibration error analysis
- [x] Calibration parameter export
- [x] Calibration parameter loading
- [ ] Essential matrix estimation
- [ ] Relative pose recovery

## Requirements

- Visual Studio 2026
- CMake
- C++17

## Run

Pass two consecutive image frames to the executable:

```powershell
out/build/local-debug/project_3_vo.exe `
    data/vo_001.JPG `
    data/vo_002.JPG `
    config/iphone13_camera.yaml
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

- Determinant: `-1.29247e-26`
- Singular values: `1.00002`, `1.27583e-05`, `2.96733e-22`
- Smallest-to-second singular value ratio: `2.3258e-17`

The near-zero third singular value confirms that the fundamental
matrix has numerical rank 2.

### Sampson error experiment

The reported values are square-root Sampson errors and can be
interpreted approximately in pixels.

| Match set | Count | Mean | Median | Maximum |
|---|---:|---:|---:|---:|
| Ratio-test matches | 205 | 4.02044 | 0.481017 | 552.436 |
| RANSAC inliers | 124 | 0.270322 | 0.268484 | 0.69123 |

RANSAC removed matches that were inconsistent with the estimated
two-view epipolar geometry.

## Camera calibration experiment

- Device: iPhone 13
- Lens: rear 1x wide camera
- Image size: `4032 x 3024`
- Chessboard inner corners: `9 x 6`
- Square size: `23.5 mm`
- Captured images: `19`
- Final calibration images: `17`
- RMS reprojection error: `1.23732 px`
- Verified RMS reprojection error: `1.23733 px`

Two images were excluded:

- `calib_019.JPG`: chessboard corners were not detected.
- `calib_014.JPG`: per-view RMS error was `3.86794 px`, making it a clear outlier.

Removing `calib_014.JPG` reduced the overall RMS error from
`1.52792 px` to `1.23732 px`, while the estimated focal lengths
changed by only about `0.05%`.

Camera matrix:

```text
[3160.117974822863, 0, 2022.709880446517;
 0, 3148.400858131528, 1524.233970948727;
 0, 0, 1]
```

Distortion coefficients `[k1, k2, p1, p2, k3]`:

```text
[0.1361488370746569,
 -0.5742277343236896,
 0.001449839819557136,
 -0.0002386635394434429,
 0.9229426820254659]
```

### Per-view reprojection error analysis

The final per-view RMS errors ranged from `0.624462 px` to
`2.17681 px`. The independently computed overall RMS differed from
the value returned by `cv::calibrateCamera` by only
`1.65706e-06 px`, confirming that the reprojection error calculation
is consistent.

### Camera calibration

Run the calibration tool with a directory containing calibration
images:

```powershell
.\out\build\local-debug\camera_calibrate.exe `
    .\data\calibration `
    .\config\iphone13_camera.yaml
```

The calibration tool exports the image size, camera matrix,
distortion coefficients, reprojection RMS error, and chessboard
metadata in OpenCV YAML format.
