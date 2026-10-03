#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

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
	//去畸变
	PointCorrespondences undistortPointCorrespondences(
		const PointCorrespondences& correspondences,
		const cv::Mat& camera_matrix,
		const cv::Mat& distortion_coefficients)
	{
		PointCorrespondences undistorted;

		if (correspondences.points1.empty()
			|| correspondences.points1.size()
			!= correspondences.points2.size()
			|| camera_matrix.empty()
			|| camera_matrix.rows != 3
			|| camera_matrix.cols != 3
			|| distortion_coefficients.empty())
		{
			return undistorted;
		}
		cv::undistortPoints(
			correspondences.points1,
			undistorted.points1,
			camera_matrix,
			distortion_coefficients,
			cv::noArray(),
			camera_matrix
		);
		cv::undistortPoints(
			correspondences.points2,
			undistorted.points2,
			camera_matrix,
			distortion_coefficients,
			cv::noArray(),
			camera_matrix
		);
		return undistorted;
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
		//fundamental must three rows and three columns
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
	SampsonErrorStatistics computeSampsonErrorStatistics(
		const cv::Mat& fundamental_matrix,
		const PointCorrespondences& correspondences)
	{
		SampsonErrorStatistics statistics;
		if (fundamental_matrix.empty()
			|| fundamental_matrix.rows != 3
			|| fundamental_matrix.cols != 3
			|| fundamental_matrix.channels() != 1
			|| correspondences.points1.empty()
			|| correspondences.points1.size()
			!= correspondences.points2.size())
		{
			return statistics;
		}
		cv::Mat matrix_64f;
		fundamental_matrix.convertTo(matrix_64f, CV_64F);
		const cv::Matx33d matrix(
			matrix_64f.at<double>(0, 0),
			matrix_64f.at<double>(0, 1),
			matrix_64f.at<double>(0, 2),
			matrix_64f.at<double>(1, 0),
			matrix_64f.at<double>(1, 1),
			matrix_64f.at<double>(1, 2),
			matrix_64f.at<double>(2, 0),
			matrix_64f.at<double>(2, 1),
			matrix_64f.at<double>(2, 2)
		);
		//求转置
		const cv::Matx33d transposed_matrix = matrix.t();
		std::vector<double> errors;
		errors.reserve(correspondences.points1.size());
		for (std::size_t index = 0;
			index < correspondences.points1.size();
			++index)
		{
			const cv::Point2f& point1 =
				correspondences.points1[index];
			const cv::Point2f& point2 =
				correspondences.points2[index];
			//转换为齐次坐标
			const cv::Vec3d homogeneous_point1(
				point1.x,
				point1.y,
				1.0
			);
			const cv::Vec3d homogeneous_point2(
				point2.x,
				point2.y,
				1.0
			);
			//极线与反方向的极线
			const cv::Vec3d line2 =
				matrix * homogeneous_point1;
			const cv::Vec3d line1 =
				transposed_matrix * homogeneous_point2;
			//点积
			const double residual =
				homogeneous_point2.dot(line2);
			const double denominator =
				line2[0] * line2[0]
				+ line2[1] * line2[1]
				+ line1[0] * line1[0]
				+ line1[1] * line1[1];
			if (denominator <= 0.0)
			{
				continue;
			}
			const double error =
				std::abs(residual) / std::sqrt(denominator);
			if (std::isfinite(error))
			{
				errors.push_back(error);
			}
		}
		if (errors.empty())
		{
			return statistics;
		}
		std::sort(errors.begin(), errors.end());
		const double sum =
			std::accumulate(errors.begin(), errors.end(), 0.0);
		statistics.count = errors.size();
		//显式转换
		statistics.mean =
			sum / static_cast<double>(statistics.count);
		statistics.maximum = errors.back();
		const std::size_t middle = statistics.count / 2;
		if (statistics.count % 2 == 0)
		{
			statistics.median =
				(errors[middle - 1] + errors[middle]) / 2.0;
		}
		else
		{
			statistics.median = errors[middle];
		}
		statistics.valid = true;
		return statistics;
	}
}		// namespace vo