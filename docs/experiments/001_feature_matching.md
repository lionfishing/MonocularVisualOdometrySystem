# Feature Matching Experiment

## Objective

Compare several ORB descriptor-matching strategies and evaluate how geometric
verification removes matches that are inconsistent with two-view geometry.

## Input

- Image 1: `data/img1.jpg`
- Image 2: `data/img2.jpg`
- Feature detector and descriptor: ORB
- Maximum features per image: `1000`
- Descriptor matcher: brute-force Hamming matcher
- Lowe ratio-test threshold: `0.75`
- Fundamental-matrix RANSAC threshold: `1.0 px`

## ORB feature extraction

| Item | Image 1 | Image 2 |
|---|---:|---:|
| Keypoints | 1000 | 1000 |
| Descriptor rows | 1000 | 1000 |
| Descriptor columns | 32 | 32 |
| Descriptor type | 0 (`CV_8U`) | 0 (`CV_8U`) |
| Bytes per descriptor | 32 | 32 |

An ORB descriptor contains 256 binary values, stored in 32 bytes.

## Raw matching

Each descriptor in image 1 was matched to its nearest descriptor in image 2.

| Statistic | Value |
|---|---:|
| Raw matches | 1000 |
| Minimum Hamming distance | 13 |
| Maximum Hamming distance | 100 |
| Mean Hamming distance | 56.684 |

Raw matching always returns a nearest descriptor, even when that nearest
descriptor is not a reliable correspondence.

## Matching-method comparison

| Method | Matches | Retention rate |
|---|---:|---:|
| Raw 1-NN | 1000 | 100.0% |
| Cross-check | 385 | 38.5% |
| KNN ratio test (`r = 0.75`) | 205 | 20.5% |

Cross-check requires the two descriptors to select each other as their best
matches.

The ratio test compares the best and second-best candidates. A match is retained
only when the best candidate is sufficiently better than the second-best
candidate.

The ratio test retained fewer matches because it rejected points whose
descriptors were similar to multiple locations. Such points are not distinctive
enough to establish an unambiguous correspondence.

## Visual inspection

The following observations were made from the match visualization:

- Match lines on the same physical objects were approximately parallel.
- A small number of crossing lines remained.
- There were no large numbers of long match lines crossing unrelated regions.
- Matches around corners and strong edges were usually correct.
- The visual difference between cross-check and ratio-test results was not
  always obvious.

Visual inspection alone cannot prove that a match satisfies the camera-motion
geometry. Geometric verification is still required.

## Fundamental matrix estimation

The 205 ratio-test matches were passed to fundamental-matrix estimation using
RANSAC.

| Ratio-test matches | RANSAC inliers | Inlier rate | Threshold |
|---:|---:|---:|---:|
| 205 | 124 | 60.4878% | 1.0 px |

Estimated fundamental matrix:

```text
[5.11823131494769e-06, 1.132017626617464e-05, -0.004752024166336392;
 -1.125949410982854e-05, 6.354910519435543e-06, -0.002976683843318251;
 0.0009120099290892425, -0.002280965151055145, 1]
```

RANSAC retained matches that were consistent with a single two-view epipolar
geometry and rejected inconsistent matches.

## Fundamental matrix diagnostics

| Diagnostic | Value |
|---|---:|
| Determinant | `-1.29247e-26` |
| Largest singular value | `1.00002` |
| Second singular value | `1.27583e-05` |
| Smallest singular value | `2.96733e-22` |
| Smallest-to-second ratio | `2.3258e-17` |

The determinant and third singular value are both close to zero. This confirms
that the estimated fundamental matrix has numerical rank 2, as required by
two-view projective geometry.

## Sampson error experiment

The reported values are square-root Sampson errors and can be interpreted
approximately in pixels.

| Match set | Count | Mean | Median | Maximum |
|---|---:|---:|---:|---:|
| All ratio-test matches | 205 | 4.02044 | 0.481017 | 552.436 |
| Fundamental-matrix RANSAC inliers | 124 | 0.270322 | 0.268484 | 0.69123 |

The RANSAC-inlier errors were much smaller and more concentrated than the errors
of the complete match set.

## Conclusion

The experiment demonstrated three different filtering levels:

1. Raw nearest-neighbour matching provides many correspondences but contains
   ambiguous and incorrect matches.
2. Cross-check and the Lowe ratio test remove descriptor-level ambiguity.
3. Fundamental-matrix RANSAC removes matches that do not satisfy a consistent
   two-view geometry.

Descriptor similarity is therefore necessary, but it is not sufficient to prove
that a feature match is geometrically correct.