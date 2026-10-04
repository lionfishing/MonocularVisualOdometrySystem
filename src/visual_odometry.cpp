#include "visual_odometry.h"
#include "geometry.h"
#include "matching.h"

namespace vo
{

    MonocularVisualOdometry::MonocularVisualOdometry(
        const CameraCalibration& calibration,
        const VisualOdometryOptions& options
    )
        // 列表初始化
        : calibration_(calibration),
        options_(options),
        // 第一帧相机坐标系定义为世界坐标系，因此初始时相机相对于世界没有旋转。
        rotation_world_from_camera_(
            cv::Mat::eye(3, 3, CV_64F)
        ),
        // 第一帧相机被定义在世界原点
        position_world_(
            cv::Mat::zeros(3, 1, CV_64F)
        )
    {
        // MAT 仍指向同一块内存，需另复制一份
        calibration_.camera_matrix =
            calibration.camera_matrix.clone();

        calibration_.distortion_coefficients =
            calibration.distortion_coefficients.clone();
    }

    VisualOdometryResult MonocularVisualOdometry::processFrame(
        const cv::Mat& grayscale_image
    )
    {
        VisualOdometryResult result;

        result.initialized = initialized_;
        result.rotation_world_from_camera =
            rotation_world_from_camera_.clone();
        result.position_world =
            position_world_.clone();

        if (grayscale_image.empty())
        {
            return result;
        }

        if (grayscale_image.type() != CV_8UC1)
        {
            return result;
        }

        if (grayscale_image.size() != calibration_.image_size)
        {
            return result;
        }

        const FeatureSet current_features =
            extractOrbFeatures(
                grayscale_image,
                options_.max_features
            );

        if (current_features.descriptors.empty())
        {
            return result;
        }

        if (!initialized_)
        {
            reference_features_ = current_features;
            initialized_ = true;

            result.initialized = true;
            return result;
        }

        // 在这里匹配上一帧与当前帧。
        const std::vector<cv::DMatch> matches =
            matchDescriptorsWithRatioTest(
                reference_features_.descriptors,
                current_features.descriptors,
                options_.ratio_threshold
            );

        result.match_count = matches.size();

        if (matches.size() < 5)
        {
            return result;
        }

        const PointCorrespondences correspondences =
            buildPointCorrespondences(
                reference_features_.keypoints,
                current_features.keypoints,
                matches
            );

        const PointCorrespondences undistorted_correspondences =
            undistortPointCorrespondences(
                correspondences,
                calibration_.camera_matrix,
                calibration_.distortion_coefficients
            );

        const EssentialMatrixResult essential_result =
            estimateEssentialMatrixRansac(
                undistorted_correspondences,
                matches,
                calibration_.camera_matrix,
                options_.ransac_threshold,
                options_.confidence
            );

        result.essential_inlier_count =
            essential_result.inlier_matches.size();

        if (essential_result.essential_matrix.empty())
        {
            return result;
        }

        const RelativePoseResult relative_pose =
            recoverRelativePose(
                essential_result,
                undistorted_correspondences,
                matches,
                calibration_.camera_matrix
            );

        if (!relative_pose.valid)
        {
            return result;
        }

        result.cheirality_inlier_count =
            relative_pose.cheirality_inlier_count;

        const double cheirality_inlier_rate =
            static_cast<double>(
                relative_pose.cheirality_inlier_count
                )
            / static_cast<double>(
                essential_result.inlier_matches.size()
                );

        const bool pose_is_reliable =
            relative_pose.cheirality_inlier_count
            >= options_.min_cheirality_inliers
            && cheirality_inlier_rate
            >= options_.min_cheirality_inlier_rate;

        if (!pose_is_reliable)
        {
            return result;
        }

        // 在这里累计 relative_pose。
        //  位移累计
        // 当前相机中心在上一帧坐标系中的位置
        const cv::Mat displacement_previous_camera =
            -relative_pose.rotation.t()
            * relative_pose.translation;
        // 把位移转换到世界坐标系并累计
        position_world_ +=
            rotation_world_from_camera_
            * displacement_previous_camera;
        // 累计相机朝向
        // 从上一帧到世界坐标系变换到这一帧到世界
        rotation_world_from_camera_ =
            rotation_world_from_camera_
            * relative_pose.rotation.t();
        // 当前帧已经成功接受，成为新的可靠参考帧
        reference_features_ = current_features;

        result.pose_updated = true;
        result.rotation_world_from_camera =
            rotation_world_from_camera_.clone();
        result.position_world =
            position_world_.clone();

        return result;
    }

}  // namespace vo