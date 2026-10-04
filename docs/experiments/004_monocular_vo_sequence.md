# Monocular VO Sequence Experiment

Date: 2026-10-04

## Input

- Device: iPhone 13
- Lens: rear 1x wide camera
- Resolution: `4032 x 3024`
- Calibration: `config/iphone13_camera.yaml`
- Number of frames: `13`
- Sequence: `data/sequence/vo_001.JPG` to `vo_013.JPG`
- ORB maximum features: `1000`
- Ratio-test threshold: `0.75`
- Essential-matrix RANSAC threshold: `1.0 px`
- RANSAC confidence: `0.99`
- Minimum cheirality inliers: `15`
- Minimum cheirality-inlier rate: `30%`

The sequence files were renamed in reverse order relative to the earlier
two-frame experiment. Comparisons with earlier results must account for this
reversal.

## Results

| Frame | Status | Matches | E inliers | Cheirality inliers | X | Y | Z |
|---|---|---:|---:|---:|---:|---:|---:|
| vo_001.JPG | INIT | - | - | - | 0 | 0 | 0 |
| vo_002.JPG | SKIP | 276 | 187 | 0 | - | - | - |
| vo_003.JPG | OK | 207 | 130 | 98 | -0.897485 | -0.340297 | -0.280569 |
| vo_004.JPG | OK | 173 | 109 | 66 | -0.0598774 | 0.0304473 | -0.681770 |
| vo_005.JPG | OK | 135 | 89 | 67 | -0.414612 | 0.211419 | -1.599060 |
| vo_006.JPG | OK | 92 | 45 | 22 | -0.773524 | 0.765408 | -2.350240 |
| vo_007.JPG | OK | 46 | 25 | 24 | -0.848459 | 1.132380 | -3.277450 |
| vo_008.JPG | OK | 44 | 28 | 22 | 0.040532 | 0.687691 | -3.386770 |
| vo_009.JPG | OK | 154 | 110 | 103 | -0.952943 | 0.748151 | -3.290060 |
| vo_010.JPG | OK | 181 | 128 | 128 | -0.430286 | 1.085860 | -4.072860 |
| vo_011.JPG | OK | 81 | 41 | 18 | 0.514103 | 0.791498 | -4.219420 |
| vo_012.JPG | OK | 164 | 82 | 82 | 1.399390 | 0.931610 | -4.662860 |
| vo_013.JPG | OK | 285 | 174 | 174 | 2.275450 | 1.116350 | -5.108270 |

## Summary

- Initialized frames: `1`
- Successful pose updates: `11`
- Rejected frames: `1`
- Rejected frame: `vo_002.JPG`
- Final unscaled position: `(2.275450, 1.116350, -5.108270)`

The trajectory is unscaled. Its coordinates must not be interpreted as metres
or centimetres.