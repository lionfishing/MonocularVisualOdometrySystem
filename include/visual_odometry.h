#pragma once

#include <cstddef>

#include <opencv2/core.hpp>

#include "camera_calibration.h"
#include "feature.h"

namespace vo 
{
    struct VisualOdometryOptions
    {
        int max_features = 1000;
        float ratio_threshold = 0.75F;
        double ransac_threshold = 1.0;
        double confidence = 0.99;

        std::size_t min_cheirality_inliers = 15;
        double min_cheirality_inlier_rate = 0.3;
    };

    struct VisualOdometryResult
    {
        bool initialized = false;
        bool pose_updated = false;

        std::size_t match_count = 0;
        std::size_t essential_inlier_count = 0;
        std::size_t cheirality_inlier_count = 0;

        cv::Mat rotation_world_from_camera;
        cv::Mat position_world;
    };

    class MonocularVisualOdometry
    {
    public:
        explicit MonocularVisualOdometry(
            const CameraCalibration& calibration,
            const VisualOdometryOptions& options = {}
        );

        VisualOdometryResult processFrame(
            const cv::Mat& grayscale_image
        );
    private:
        CameraCalibration calibration_;
        VisualOdometryOptions options_;

        FeatureSet reference_features_;
        bool initialized_ = false;

        cv::Mat rotation_world_from_camera_;
        cv::Mat position_world_;
    };

}//namespace vo
