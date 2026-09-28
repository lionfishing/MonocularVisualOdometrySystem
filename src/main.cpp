#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

#include "feature.h"

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
    cv::BFMatcher matcher(
        cv::NORM_HAMMING, 
        false
    );
    cv::BFMatcher cross_check_matcher(
        cv::NORM_HAMMING,
        true
    );
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> cross_check_matches;
    std::vector<std::vector<cv::DMatch>> knn_matches;

    matcher.match(
        features1.descriptors,
        features2.descriptors,
        matches
    );
    cross_check_matcher.match(
        features1.descriptors,
        features2.descriptors,
        cross_check_matches
    );
    matcher.knnMatch(
        features1.descriptors,
        features2.descriptors,
        knn_matches,
        2
    );

    std::vector<cv::DMatch> ratio_matches;
    for (const std::vector<cv::DMatch>& candidates : knn_matches)
    {
        if (candidates.size() < 2)
        {
            continue;
        }
        const cv::DMatch& best_match = candidates[0];
        const cv::DMatch& second_match = candidates[1];

        if (best_match.distance
            < kRatioThreshold * second_match.distance)
        {
            ratio_matches.push_back(best_match);
        }
    }
    if (matches.empty())
    {
        std::cerr << "No descriptor matches were found.\n";
        return 6;
    }
    if (cross_check_matches.empty())
    {
        std::cerr << "No cross-check matches were found.\n";
        return 7;
    }
    if (ratio_matches.empty())
    {
        std::cerr << "No ratio-test matches were found.\n";
        return 8;
    }

    std::cout << "Raw matches: "
        << matches.size() << '\n';

    std::cout << "KNN query groups: "
        << knn_matches.size() << '\n';
    std::cout << "Ratio-test matches: "
        << ratio_matches.size() << '\n';
    std::cout << "Ratio-test retention rate: "
        << 100.0 * static_cast<double>(ratio_matches.size())
        / static_cast<double>(knn_matches.size())
        << "%\n";

    std::sort(
        matches.begin(),
        matches.end(),
        [](const cv::DMatch& lhs, const cv::DMatch& rhs)
        {
            return lhs.distance < rhs.distance;
        }
    );
    std::sort(
        cross_check_matches.begin(),
        cross_check_matches.end(),
        [](const cv::DMatch& lhs, const cv::DMatch& rhs)
        {
            return lhs.distance < rhs.distance;
        }
    );
    std::sort(
        ratio_matches.begin(),
        ratio_matches.end(),
        [](const cv::DMatch& lhs, const cv::DMatch& rhs)
        {
            return lhs.distance < rhs.distance;
        }
    );


    double distance_sum = 0.0;
    for (const cv::DMatch& match : matches)
    {
        distance_sum += match.distance;
    }
    const double mean_distance =
        distance_sum / static_cast<double>(matches.size());

    std::cout << "Minimum distance: "
        << matches.front().distance << '\n';
    std::cout << "Maximum distance: "
        << matches.back().distance << '\n';
    std::cout << "Mean distance: "
        << mean_distance << '\n';

    std::cout << "Cross-check matches: "
        << cross_check_matches.size() << '\n';
    std::cout << "Cross-check retention rate: "
        << 100.0 * static_cast<double>(cross_check_matches.size())
        / static_cast<double>(matches.size())
        << "%\n";


    const std::size_t match_count_to_draw =
        std::min<std::size_t>(100, matches.size());
    const std::vector<cv::DMatch> selected_matches(
        matches.begin(),
        matches.begin() + match_count_to_draw
    );

    const std::size_t cross_check_count_to_draw =
        std::min<std::size_t>(
            100,
            cross_check_matches.size()
        );
    const std::vector<cv::DMatch> selected_cross_check_matches(
        cross_check_matches.begin(),
        cross_check_matches.begin() + cross_check_count_to_draw
    );

    const std::size_t ratio_count_to_draw =
        std::min<std::size_t>(
            100,
            ratio_matches.size()
        );
    const std::vector<cv::DMatch> selected_ratio_matches(
        ratio_matches.begin(),
        ratio_matches.begin() + ratio_count_to_draw
    );

    cv::Mat matches_view;
    cv::drawMatches(
        image1,
        features1.keypoints,
        image2,
        features2.keypoints,
        selected_matches,
        matches_view,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    cv::Mat cross_check_view;
    cv::drawMatches(
        image1,
        features1.keypoints,
        image2,
        features2.keypoints,
        selected_cross_check_matches,
        cross_check_view,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
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

    
    cv::imshow("Selected ORB matches", matches_view);
    cv::imshow("Cross-check ORB matches", cross_check_view);
    cv::imshow("Ratio-test ORB matches", ratio_view);

    cv::waitKey(0);
    return 0;
}
