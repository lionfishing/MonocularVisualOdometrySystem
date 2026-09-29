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
FundamentalMatrixDiagnostics analyzeFundamentalMatrix(
	const cv::Mat& fundamental_matrix)
{
	FundamentalMatrixDiagnostics diagnostics;
	//fundamental must three rows and three cows 
	if (fundamental_matrix.empty()
		|| fundamental_matrix.rows != 3
		|| fundamental_matrix.cols != 3
		|| fundamental_matrix.channels() != 1)
	{
		return diagnostics;
	}
	cv::Mat matrix_64f;
	//转换到64精度
	fundamental_matrix.convertTo(
		matrix_64f,
		CV_64F);
	cv::Mat singular_values;
	cv::SVD::compute(
		matrix_64f,
		singular_values
	);
	
	if (singular_values.total() != 3)
	{
		return diagnostics;
	}
	//计算行列式
	diagnostics.determinant =
		cv::determinant(matrix_64f);
	for (int i = 0; i < 3; i++)
	{
		diagnostics.singular_values[i] =
			singular_values.at<double>(i, 0);
	}
	//进行除法要确保非零
	if (diagnostics.singular_values[1] > 0.0)
	{
		diagnostics.smallest_to_second_ratio =
			diagnostics.singular_values[2]
			/ diagnostics.singular_values[1];
	}
	diagnostics.valid = true;
	return diagnostics;
}

}		// namespace vo