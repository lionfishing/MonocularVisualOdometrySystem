#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace vo
{
    std::vector<cv::DMatch> matchDescriptorsWithRatioTest(
        const cv::Mat& query_descriptors,
        const cv::Mat& train_descriptors,
        float ratio_threshold
    );

}// namespace vo