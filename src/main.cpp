#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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

    std::cout << "Ratio-test matches: "
        << ratio_matches.size() << '\n';

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
    const double inlier_ratio =
        static_cast<double>(
            fundamental_result.inlier_matches.size()
            )
        / static_cast<double>(ratio_matches.size());
    std::cout << "Fundamental matrix:\n"
        << fundamental_result.fundamental_matrix
        << '\n';
    std::cout << "RANSAC inliers: "
        << fundamental_result.inlier_matches.size()
        << " / " << ratio_matches.size()
        << '\n';
    std::cout << "RANSAC inlier ratio: "
        << 100.0 * inlier_ratio
        << "%\n";
    const std::size_t ratio_count_to_draw =
        std::min<std::size_t>(
            100,
            ratio_matches.size()
        );
    const std::size_t ransac_count_to_draw =
        std::min<std::size_t>(
            100,
            fundamental_result.inlier_matches.size()
        );
    const std::vector<cv::DMatch> selected_ratio_matches(
        ratio_matches.begin(),
        ratio_matches.begin() + ratio_count_to_draw
    );
    const std::vector<cv::DMatch> selected_ransac_matches(
        fundamental_result.inlier_matches.begin(),
        fundamental_result.inlier_matches.begin() + ransac_count_to_draw
    );
    const vo::FundamentalMatrixDiagnostics diagnostics =
        vo::analyzeFundamentalMatrix(
            fundamental_result.fundamental_matrix
        );
    if (!diagnostics.valid)
    {
        std::cerr
            << "Failed to analyze the fundamental matrix.\n";
        return 10;
    }

    std::cout << "Fundamental matrix determinant: "
        << diagnostics.determinant
        << '\n';

    std::cout << "Fundamental matrix singular values: "
        << diagnostics.singular_values[0] << ", "
        << diagnostics.singular_values[1] << ", "
        << diagnostics.singular_values[2] << '\n';

    std::cout << "Smallest-to-second singular value ratio: "
        << diagnostics.smallest_to_second_ratio
        << '\n';

    /*cv::Mat ratio_view;
    cv::drawMatches(
        image1,
        features1.keypoints,
        image2,
        features2.keypoints,
        selected_ratio_matches,
        ratio_view,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    cv::Mat inlier_view;
    cv::drawMatches(
        image1,
        features1.keypoints,
        image2,
        features2.keypoints,
        selected_ransac_matches,
        inlier_view,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    cv::imshow(
        "Ratio-test ORB matches", 
        ratio_view);
    cv::imshow(
        "Fundamental matrix RANSAC inliers",
        inlier_view
    );*/

    cv::waitKey(0);
    return 0;
}
