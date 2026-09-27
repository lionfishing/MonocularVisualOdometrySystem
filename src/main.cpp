#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

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
    const cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;

    cv::Mat descriptors1;
    cv::Mat descriptors2;

    orb->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

    if (descriptors1.empty())
    {
        std::cerr << "Failed to compute descriptors for image 1.\n";
        return 4;
    }

    if (descriptors2.empty())
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

    matcher.match(
        descriptors1,
        descriptors2,
        matches
    );
    cross_check_matcher.match(
        descriptors1,
        descriptors2,
        cross_check_matches
    );
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

    std::cout << "Raw matches: "
        << matches.size() << '\n';

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

    cv::Mat matches_view;
    cv::drawMatches(
        image1,
        keypoints1,
        image2,
        keypoints2,
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
        keypoints1,
        image2,
        keypoints2,
        selected_cross_check_matches,
        cross_check_view,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );

    cv::imshow("Selected ORB matches", matches_view);
    cv::imshow("Cross-check ORB matches", cross_check_view);

    cv::waitKey(0);
    return 0;
}
