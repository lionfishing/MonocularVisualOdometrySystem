#pragma once

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

}		// namespace vo