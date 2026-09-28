#include "geometry.h"
#include <opencv2/calib3d.hpp>

namespace vo
{
PointCorrespondences buildPointCorrespondences(
	const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2,
	const std::vector<cv::DMatch>& matches)
{
	PointCorrespondences correspondences;
	correspondences.points1.reserve(matches.size());
	correspondences.points2.reserve(matches.size());

	for (const cv::DMatch& match : matches)
	{
		correspondences.points1.push_back(
			keypoints1[match.queryIdx].pt);
		correspondences.points2.push_back(
			keypoints2[match.trainIdx].pt);
	}
	return correspondences;
}
FundamentalMatrixResult estimateFundamentalMatrixRansac(
	const PointCorrespondences& correspondences,
	const std::vector<cv::DMatch>& matches,
	double ransac_threshold,
	double confidence)
{
	FundamentalMatrixResult result;
	if (correspondences.points1.size()
		!= correspondences.points2.size()
		|| correspondences.points1.size() != matches.size()
		|| matches.size() < 8)
	{
		return result;
	}
	std::vector<unsigned char> inlier_mask;
	result.fundamental_matrix = cv::findFundamentalMat(
		correspondences.points1,
		correspondences.points2,
		cv::FM_RANSAC,
		ransac_threshold,
		confidence,
		inlier_mask
	);
	if (result.fundamental_matrix.empty()
		|| inlier_mask.size() != matches.size())
	{
		result.fundamental_matrix.release();
		return result;
	}
	result.inlier_matches.reserve(matches.size());
	for (std::size_t index = 0;
		index < matches.size();
		++index)
	{
		if (inlier_mask[index] != 0)
		{
			result.inlier_matches.push_back(matches[index]);
		}
	}
	return result;
}
}    // namespace vo