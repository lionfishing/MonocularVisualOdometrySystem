# Two-View Geometry Experiment

## Objective

Recover the relative rotation and translation direction between two calibrated
iPhone 13 images using matched ORB features, point undistortion, essential-matrix
estimation, and pose recovery.

## Input

- Image 1: `data/vo_001.JPG`
- Image 2: `data/vo_002.JPG`
- Device: iPhone 13
- Lens: rear 1x wide camera
- Image resolution: `4032 x 3024`
- Calibration: `config/iphone13_camera.yaml`
- ORB maximum features: `1000`
- Ratio-test threshold: `0.75`
- Essential-matrix RANSAC threshold: `1.0 px`
- RANSAC confidence: `0.99`

Both input images had the same resolution as the calibration images.

## Processing pipeline

```text
ORB feature extraction
    -> KNN descriptor matching
    -> Lowe ratio test
    -> matched-point construction
    -> point undistortion using K and distortion coefficients
    -> essential-matrix estimation with RANSAC
    -> relative-pose recovery
    -> cheirality verification
```

The matched pixel coordinates were undistorted before estimating the essential
matrix. The output coordinates remained expressed in ideal pinhole-camera pixel
coordinates because the camera matrix was supplied as the projection matrix to
`cv::undistortPoints`.

## Matching and geometric verification

| Item | Result |
|---|---:|
| Ratio-test matches | 134 |
| Fundamental-matrix RANSAC inliers | 70 |
| Essential-matrix RANSAC inliers | 73 |
| Essential-matrix inlier rate | 54.4776% |

The fundamental matrix was used as an earlier uncalibrated geometric experiment.
The final calibrated pose pipeline used the essential matrix.

## Essential matrix

Estimated essential matrix:

```text
[0.1917976976555466, -0.6747262547736026, -0.07409789583212199;
 0.6765495421560209, 0.1919717554456674, -0.03883567707970882;
 -0.04672464210437276, -0.06481221482858684, -0.001778499662729286]
```

Independent numerical diagnostics:

| Diagnostic | Value |
|---|---:|
| Determinant | `-8.89e-17` |
| First singular value | `0.7071067812` |
| Second singular value | `0.7071067812` |
| Third singular value | `6.585e-10` |

An ideal essential matrix has two equal nonzero singular values and one zero
singular value. The estimated matrix closely satisfied this structure.

## Recovered relative pose

Recovered rotation matrix:

```text
[0.9507883999531783, 0.283900247225928, -0.1241050689518051;
 -0.2576579054476741, 0.9469254333868843, 0.1922098524149544;
 0.1720866708225619, -0.1507742459112899, 0.973474860741195]
```

Recovered translation direction:

```text
[-0.07025135054178819;
  0.08853645921705158;
  0.9935924934983971]
```

| Diagnostic | Result |
|---|---:|
| Translation norm | `1` |
| Cheirality inliers | `53 / 73` |
| Cheirality-inlier rate | `72.60%` |
| Rotation determinant | approximately `1` |
| Maximum error in `R^T R` | `4.44e-16` |
| Approximate rotation angle | `20.68 degrees` |

The rotation determinant and orthogonality check confirmed that the recovered
rotation was a valid numerical rotation matrix.

## Transformation interpretation

`cv::recoverPose` returns a transformation of the form:

```text
X_current = R * X_previous + t
```

The returned translation is part of the coordinate transformation. The current
camera centre expressed in the previous camera coordinate system is therefore:

```text
C_current_in_previous = -R^T * t
```

For this experiment, the corresponding direction was approximately:

```text
[-0.08138;
  0.08592;
 -0.99297]
```

## Scale limitation

The norm of the recovered translation is 1 because monocular two-view geometry
recovers only the translation direction.

The result does not determine whether the camera moved:

```text
0.1 metres
1 metre
or 10 metres
```

Additional scale information would be required to recover metric translation.

## Conclusion

The calibrated two-view pipeline successfully recovered:

- A valid essential matrix;
- A numerically valid rotation matrix;
- A unit translation direction;
- 53 correspondences satisfying both the essential-matrix model and the
  positive-depth constraint.

This experiment provides the relative-pose component later reused by the
multi-frame monocular visual-odometry class.