# Camera Calibration Experiment

## Objective

Estimate the intrinsic parameters and lens-distortion coefficients of an
iPhone 13 rear 1x wide camera, and verify the calibration through per-view and
overall reprojection errors.

## Calibration setup

- Device: iPhone 13
- Lens: rear 1x wide camera
- Image resolution: `4032 x 3024`
- Chessboard inner corners: `9 x 6`
- Measured square size: `23.5 mm`
- Originally captured images: `19`
- Final accepted calibration images: `17`
- Calibration output: `config/iphone13_camera.yaml`

The phone settings, lens selection, image resolution, focus behaviour, and image
orientation were kept consistent across the calibration and VO experiments.

## Calibration image validation

Initial validation result:

| Item | Count |
|---|---:|
| Total images | 19 |
| Chessboard detected | 18 |
| Chessboard not detected | 1 |

`calib_019.JPG` was rejected because the complete `9 x 6` inner-corner pattern
could not be detected.

All accepted images had:

```text
Detected corners: 54
```

The 54 corners correspond to:

```text
9 × 6 = 54
```

After chessboard detection, `cv::cornerSubPix` refined the corner coordinates
from pixel-level estimates to subpixel positions.

## Initial calibration result

Calibration with the 18 successfully detected images produced:

| Item | Value |
|---|---:|
| RMS reprojection error | `1.52792 px` |
| Estimated camera poses | 18 |

Initial camera matrix:

```text
[3158.447972955243, 0, 2031.446013583559;
 0, 3146.700044318764, 1539.971050113009;
 0, 0, 1]
```

Initial distortion coefficients `[k1, k2, p1, p2, k3]`:

```text
[0.1422258810211766;
 -0.6088966037280547;
 0.004046391250896971;
 0.0002083226523648999;
 0.9824825294254116]
```

## Initial per-view reprojection errors

| Image | RMS error |
|---|---:|
| calib_001.JPG | 1.43029 px |
| calib_002.JPG | 0.760918 px |
| calib_003.JPG | 0.610186 px |
| calib_004.JPG | 0.848023 px |
| calib_005.JPG | 0.849890 px |
| calib_006.JPG | 0.799318 px |
| calib_007.JPG | 0.826217 px |
| calib_008.JPG | 1.133990 px |
| calib_009.JPG | 1.470830 px |
| calib_010.JPG | 1.095490 px |
| calib_011.JPG | 1.254350 px |
| calib_012.JPG | 1.511580 px |
| calib_013.JPG | 1.860120 px |
| calib_014.JPG | 3.867940 px |
| calib_015.JPG | 1.301900 px |
| calib_016.JPG | 1.783560 px |
| calib_017.JPG | 1.232030 px |
| calib_018.JPG | 1.714340 px |

`calib_014.JPG` had a substantially larger error than the other images and was
treated as a calibration outlier.

## Final calibration result

After removing `calib_014.JPG`, calibration was repeated using 17 images.

| Item | Value |
|---|---:|
| RMS reprojection error | `1.23732 px` |
| Independently verified RMS | `1.23733 px` |
| RMS verification difference | `1.65706e-06 px` |
| Estimated camera poses | 17 |

Final camera matrix:

```text
[3160.117974822863, 0, 2022.709880446517;
 0, 3148.400858131528, 1524.233970948727;
 0, 0, 1]
```

This corresponds to approximately:

```text
fx = 3160.117974822863
fy = 3148.400858131528
cx = 2022.709880446517
cy = 1524.233970948727
```

Final distortion coefficients `[k1, k2, p1, p2, k3]`:

```text
[0.1361488370746569;
 -0.5742277343236896;
 0.001449839819557136;
 -0.0002386635394434429;
 0.9229426820254659]
```

## Final per-view reprojection errors

| Image | RMS error |
|---|---:|
| calib_001.JPG | 1.212110 px |
| calib_002.JPG | 0.716216 px |
| calib_003.JPG | 0.624853 px |
| calib_004.JPG | 0.747735 px |
| calib_005.JPG | 0.759484 px |
| calib_006.JPG | 0.624462 px |
| calib_007.JPG | 0.721199 px |
| calib_008.JPG | 1.074950 px |
| calib_009.JPG | 1.326090 px |
| calib_010.JPG | 1.065240 px |
| calib_011.JPG | 1.149180 px |
| calib_012.JPG | 1.412540 px |
| calib_013.JPG | 2.176810 px |
| calib_015.JPG | 1.345630 px |
| calib_016.JPG | 1.768830 px |
| calib_017.JPG | 1.283460 px |
| calib_018.JPG | 1.708230 px |

The final per-view errors ranged from `0.624462 px` to `2.17681 px`.

## Effect of removing the outlier

Removing `calib_014.JPG` reduced the overall RMS error:

```text
1.52792 px -> 1.23732 px
```

The estimated focal lengths changed by only approximately `0.05%`, indicating
that the calibration remained stable while the reprojection consistency
improved.

## Calibration tool

The calibration executable is run with:

```powershell
.\out\build\local-debug\camera_calibrate.exe `
    .\data\calibration `
    .\config\iphone13_camera.yaml
```

It performs:

```text
load calibration images
    -> detect chessboard corners
    -> subpixel corner refinement
    -> camera calibration
    -> per-view reprojection-error calculation
    -> calibration YAML export
```

## Conclusion

The final calibration used 17 valid images and produced an overall reprojection
RMS of approximately `1.24 px`.

The independently computed RMS agreed with the value returned by
`cv::calibrateCamera`, confirming that the reprojection-error implementation was
consistent.

The exported YAML file is the calibration source used by the later two-view
geometry and monocular visual-odometry experiments.