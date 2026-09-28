#include "matching.h"

namespace vo
{

    std::vector<cv::DMatch> matchDescriptorsWithRatioTest(
        const cv::Mat& query_descriptors,
        const cv::Mat& train_descriptors,
        float ratio_threshold)
    {
        if (query_descriptors.empty()
            || train_descriptors.empty())
        {
            return {};
        }

        cv::BFMatcher matcher(
            cv::NORM_HAMMING,
            false
        );

        std::vector<std::vector<cv::DMatch>> knn_matches;

        matcher.knnMatch(
            query_descriptors,
            train_descriptors,
            knn_matches,
            2
        );

        std::vector<cv::DMatch> ratio_matches;
        ratio_matches.reserve(knn_matches.size());

        for (const std::vector<cv::DMatch>& candidates : knn_matches)
        {
            if (candidates.size() < 2)
            {
                continue;
            }

            const cv::DMatch& best_match = candidates[0];
            const cv::DMatch& second_best_match = candidates[1];

            if (best_match.distance
                < ratio_threshold * second_best_match.distance)
            {
                ratio_matches.push_back(best_match);
            }
        }

        return ratio_matches;
    }

}  // namespace vo