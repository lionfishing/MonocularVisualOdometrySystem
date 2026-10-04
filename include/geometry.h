#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace vo
{
	struct PointCorrespondences
	{
		std::vector<cv::Point2f> points1;
		std::vector<cv::Point2f> points2;
	};
	struct FundamentalMatrixResult
	{
		cv::Mat fundamental_matrix;
		std::vector<cv::DMatch> inlier_matches;
	};
	struct EssentialMatrixResult
	{
		cv::Mat essential_matrix;
		std::vector<unsigned char> inlier_mask;
		std::vector<cv::DMatch> inlier_matches;
	};
	struct RelativePoseResult
	{
		bool valid = false;

		cv::Mat rotation;
		cv::Mat translation;

		//同时满足本质矩阵约束和正深度约束的匹配
		std::vector<unsigned char> inlier_mask;
		std::vector<cv::DMatch> inlier_matches;

		std::size_t cheirality_inlier_count = 0;
	};
	struct FundamentalMatrixDiagnostics
	{
		bool valid = false;
		double determinant = 0.0;
		cv::Vec3d singular_values{ 0.0, 0.0, 0.0 };
		double smallest_to_second_ratio = 0.0;
	};
	struct SampsonErrorStatistics
	{
		bool valid = false;
		std::size_t count = 0;
		double mean = 0.0;
		double median = 0.0;
		double maximum = 0.0;
	};

	PointCorrespondences buildPointCorrespondences(
		const std::vector<cv::KeyPoint>& keypoints1,
		const std::vector<cv::KeyPoint>& keypoints2,
		const std::vector<cv::DMatch>& matches
	);
	PointCorrespondences undistortPointCorrespondences(
		const PointCorrespondences& correspondences,
		const cv::Mat& camera_matrix,
		const cv::Mat& distortion_coefficients
	);
	EssentialMatrixResult estimateEssentialMatrixRansac(
		const PointCorrespondences& undistorted_correspondences,
		const std::vector<cv::DMatch>& matches,
		const cv::Mat& camera_matrix,
		double ransac_threshold,
		double confidence
	);
	RelativePoseResult recoverRelativePose(
		const EssentialMatrixResult& essential_result,
		const PointCorrespondences& undistorted_correspondences,
		const std::vector<cv::DMatch>& matches,
		const cv::Mat& camera_matrix
	);
	FundamentalMatrixResult estimateFundamentalMatrixRansac(
		const PointCorrespondences& correspondences,
		const std::vector<cv::DMatch>& matches,
		double ransac_threshold,
		double confidence
	);
	FundamentalMatrixDiagnostics analyzeFundamentalMatrix(
		const cv::Mat& fundamental_matrix
	);
	SampsonErrorStatistics computeSampsonErrorStatistics(
		const cv::Mat& fundamental_matrix,
		const PointCorrespondences& correspondences
	);

}  // namespace vo

