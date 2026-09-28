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
    if (correspondences.points1.size() < 8)
    {
        std::cerr
            << "Not enough point correspondences "
            << "for fundamental matrix estimation.\n";//minmum of four parts of points
        return 8;
    }
    std::cout << "Point correspondences: "
        << correspondences.points1.size() << '\n';

    const std::size_t ratio_count_to_draw =
        std::min<std::size_t>(
            100,
            ratio_matches.size()
        );
    const std::vector<cv::DMatch> selected_ratio_matches(
        ratio_matches.begin(),
        ratio_matches.begin() + ratio_count_to_draw
    );

    cv::Mat ratio_view;
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



    cv::imshow("Ratio-test ORB matches", ratio_view);

    cv::waitKey(0);
    return 0;
}
