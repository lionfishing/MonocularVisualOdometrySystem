# Project 3: Simplified Monocular Visual Odometry

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
- [x] Essential matrix estimation
- [x] Relative pose recovery
- [x] Relative-pose quality gating
- [x] Multi-frame image sequence loading
- [x] Stateful monocular visual odometry class
- [x] Global pose accumulation
- [x] Rejection of unreliable frames
- [ ] Trajectory CSV export
- [ ] 2D trajectory visualization

## Requirements

- Visual Studio 2026
- CMake
- C++17

## Run

The visual odometry executable processes an ordered image directory:

```powershell
.\out\build\local-debug\project_3_vo.exe `
    .\data\sequence `
    .\config\iphone13_camera.yaml
```

## Simplified monocular visual odometry pipeline

```text
Ordered grayscale images
    -> ORB feature extraction
    -> KNN descriptor matching
    -> Lowe ratio test
    -> point undistortion
    -> essential matrix estimation with RANSAC
    -> relative pose recovery
    -> cheirality quality gate
    -> unscaled global pose accumulation
```

A frame is accepted only when:

- The number of cheirality inliers is at least `15`.
- The cheirality-inlier rate is at least `30%`.

If a frame is rejected, the last successfully accepted frame remains the
reference frame.

## Current limitations

- Translation from `recoverPose` has direction but no metric scale.
- Every accepted translation is currently treated as having unit length.
- The reported trajectory therefore shows only approximate shape and direction.
- There is no bundle adjustment, keyframe management, loop closure, or scale
  recovery.
- A short image sequence is used as a learning experiment rather than as a
  production-quality VO benchmark.


## Experiment records

- [Feature matching](docs/experiments/001_feature_matching.md)
- [Camera calibration](docs/experiments/002_camera_calibration.md)
- [Two-view geometry](docs/experiments/003_two_view_geometry.md)
- [Monocular VO sequence](docs/experiments/004_monocular_vo_sequence.md)


## Camera calibration tool

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
