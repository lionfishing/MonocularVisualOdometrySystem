#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

#include "feature.h"
#include "matching.h"
#include "geometry.h"

constexpr float kRatioThreshold = 0.75F;
constexpr int kMaxFeatures = 1000;
constexpr double kFundamentalRansacThreshold = 1.0;
constexpr double kRansacConfidence = 0.99;

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr
            << "Usage: project_3_vo <image1_path> <image2_path>\n";
        return 1;
    }
    
    const cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    const cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (image1.empty())
    {
        std::cerr << "Failed to load image: " << argv[1] << '\n';
        return 2;
    }
    if (image2.empty())
    {
        std::cerr << "Failed to load image: " << argv[2] << '\n';
        return 3;
    }

    const vo::FeatureSet features1 =
        vo::extractOrbFeatures(image1, kMaxFeatures);
    const vo::FeatureSet features2 =
        vo::extractOrbFeatures(image2, kMaxFeatures);
 
    if (features1.descriptors.empty())
    {
        std::cerr << "Failed to compute descriptors for image 1.\n";
        return 4;
    }
    if (features2.descriptors.empty())
    {
        std::cerr << "Failed to compute descriptors for image 2.\n";
        return 5;
    }

    /*描述子一致性是局部外观约束，不是几何正确性的证明。*/
    std::vector<cv::DMatch> ratio_matches =
        vo::matchDescriptorsWithRatioTest(
            features1.descriptors,
            features2.descriptors,
            kRatioThreshold
        );

    if (ratio_matches.empty())
    {
        std::cerr << "No ratio-test matches were found.\n";
        return 6;
    }
    std::sort(
        ratio_matches.begin(),
        ratio_matches.end(),
        [](const cv::DMatch& lhs, const cv::DMatch& rhs)
        {
            return lhs.distance < rhs.distance;
        }
    );
    const vo::PointCorrespondences correspondences =
        vo::buildPointCorrespondences(
            features1.keypoints,
            features2.keypoints,
            ratio_matches
        );
    if (correspondences.points1.size()
        != correspondences.points2.size())
    {
        std::cerr << "Point correspondence sizes do not match.\n";
        return 7;
    }
    //The normalized eight-point algorithm requires at least eight correspondences.
    if (correspondences.points1.size() < 8)
    {
        std::cerr
            << "Not enough point correspondences "
            << "for fundamental matrix estimation.\n";
        return 8;
    }

    const vo::FundamentalMatrixResult fundamental_result =
        vo::estimateFundamentalMatrixRansac(
            correspondences,
            ratio_matches,
            kFundamentalRansacThreshold,
            kRansacConfidence
        );
    if (fundamental_result.fundamental_matrix.empty())
    {
        std::cerr << "Failed to estimate the fundamental matrix.\n";
        return 9;
    }
    const vo::PointCorrespondences inlier_correspondences =
        vo::buildPointCorrespondences(
            features1.keypoints,
            features2.keypoints,
            fundamental_result.inlier_matches
        );

    const vo::SampsonErrorStatistics all_statistics =
        vo::computeSampsonErrorStatistics(
            fundamental_result.fundamental_matrix,
            correspondences
        );
    const vo::SampsonErrorStatistics inlier_statistics =
        vo::computeSampsonErrorStatistics(
            fundamental_result.fundamental_matrix,
            inlier_correspondences
        );
    if (!all_statistics.valid || !inlier_statistics.valid)
    {
        std::cerr << "Failed to compute Sampson error statistics.\n";
        return 10;
    }
    std::cout
        << "All matches Sampson error:\n"
        << "  Count: " << all_statistics.count << '\n'
        << "  Mean: " << all_statistics.mean << '\n'
        << "  Median: " << all_statistics.median << '\n'
        << "  Maximum: " << all_statistics.maximum << '\n';
    std::cout
        << "RANSAC inliers Sampson error:\n"
        << "  Count: " << inlier_statistics.count << '\n'
        << "  Mean: " << inlier_statistics.mean << '\n'
        << "  Median: " << inlier_statistics.median << '\n'
        << "  Maximum: " << inlier_statistics.maximum << '\n';

    return 0;
}
