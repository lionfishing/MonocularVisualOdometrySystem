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

}		// namespace vo
