#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

#include "feature.h"
#include "matching.h"
#include "geometry.h"
#include "camera_calibration.h"

constexpr float kRatioThreshold = 0.75F;
constexpr int kMaxFeatures = 1000;
constexpr double kFundamentalRansacThreshold = 1.0;
//几何一致性允许的误差范围
constexpr double kEssentialRansacThreshold = 1.0;
//RANSAC 希望达到的模型估计置信度
constexpr double kRansacConfidence = 0.99;
//位姿质量门控
constexpr std::size_t kMinCheiralityInliers = 15;//数量
constexpr double kMinCheiralityInlierRate = 0.3; //比例

int main(int argc, char* argv[])
{
	if (argc != 4)
	{
		std::cerr
			<< "Usage: project_3_vo "
			<< "<image1_path> "
			<< "<image2_path> "
			<< "<calibration_file>\n";
		return 1;
	}

	const cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	const cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

	if (image1.empty())
	{
		std::cerr << "Failed to load image: " << argv[1] << '\n';
		return 2;
	}
	if (image2.empty())
	{
		std::cerr << "Failed to load image: " << argv[2] << '\n';
		return 3;
	}
	if (image1.size() != image2.size())
	{
		std::cerr
			<< "Input image sizes do not match:\n"
			<< "  Image 1: "
			<< image1.cols << " x " << image1.rows << '\n'
			<< "  Image 2: "
			<< image2.cols << " x " << image2.rows << '\n';
		return 12;
	}
	//相机内参读入
	vo::CameraCalibration calibration;
	if (!vo::loadCameraCalibration(
		argv[3],
		calibration
	))
	{
		std::cerr
			<< "Failed to load camera calibration: "
			<< argv[3]
			<< "\n";
		return 11;
	}

	if (image1.size() != calibration.image_size)
	{
		std::cerr
			<< "Input image size does not match calibration:\n"
			<< "  Input: "
			<< image1.cols << " x " << image1.rows << '\n'
			<< "  Calibration: "
			<< calibration.image_size.width
			<< " x "
			<< calibration.image_size.height
			<< '\n';
		return 13;
	}

	const vo::FeatureSet features1 =
		vo::extractOrbFeatures(image1, kMaxFeatures);
	const vo::FeatureSet features2 =
		vo::extractOrbFeatures(image2, kMaxFeatures);

	if (features1.descriptors.empty())
	{
		std::cerr << "Failed to compute descriptors for image 1.\n";
		return 4;
	}
	if (features2.descriptors.empty())
	{
		std::cerr << "Failed to compute descriptors for image 2.\n";
		return 5;
	}

	/*描述子一致性是局部外观约束，不是几何正确性的证明。*/
	std::vector<cv::DMatch> ratio_matches =
		vo::matchDescriptorsWithRatioTest(
			features1.descriptors,
			features2.descriptors,
			kRatioThreshold
		);

	if (ratio_matches.empty())
	{
		std::cerr << "No ratio-test matches were found.\n";
		return 6;
	}
	std::sort(
		ratio_matches.begin(),
		ratio_matches.end(),
		[](const cv::DMatch& lhs, const cv::DMatch& rhs)
		{
			return lhs.distance < rhs.distance;
		}
	);
	const vo::PointCorrespondences correspondences =
		vo::buildPointCorrespondences(
			features1.keypoints,
			features2.keypoints,
			ratio_matches
		);
	if (correspondences.points1.size()
		!= correspondences.points2.size())
	{
		std::cerr << "Point correspondence sizes do not match.\n";
		return 7;
	}
	//The normalized eight-point algorithm requires at least eight correspondences.
	if (correspondences.points1.size() < 8)
	{
		std::cerr
			<< "Not enough point correspondences "
			<< "for fundamental matrix estimation.\n";
		return 8;
	}
	const vo::PointCorrespondences undistorted_correspondences =
		vo::undistortPointCorrespondences(
			correspondences,
			calibration.camera_matrix,
			calibration.distortion_coefficients
		);

	if (undistorted_correspondences.points1.size()
		!= correspondences.points1.size()
		|| undistorted_correspondences.points2.size()
		!= correspondences.points2.size())
	{
		std::cerr
			<< "Failed to undistort point correspondences.\n";
		return 14;
	}

					//***对照实验***//
	const vo::FundamentalMatrixResult fundamental_result =
		vo::estimateFundamentalMatrixRansac(
			correspondences,
			ratio_matches,
			kFundamentalRansacThreshold,
			kRansacConfidence
		);
	if (fundamental_result.fundamental_matrix.empty())
	{
		std::cerr << "Failed to estimate the fundamental matrix.\n";
		return 9;
	}
	const vo::PointCorrespondences inlier_correspondences =
		vo::buildPointCorrespondences(
			features1.keypoints,
			features2.keypoints,
			fundamental_result.inlier_matches
		);
	//重投影误差分析 对照组 分别计算经过ransac组和普通组
	const vo::SampsonErrorStatistics all_statistics =
		vo::computeSampsonErrorStatistics(
			fundamental_result.fundamental_matrix,
			correspondences
		);
	const vo::SampsonErrorStatistics inlier_statistics =
		vo::computeSampsonErrorStatistics(
			fundamental_result.fundamental_matrix,
			inlier_correspondences
		);
	if (!all_statistics.valid || !inlier_statistics.valid)
	{
		std::cerr << "Failed to compute Sampson error statistics.\n";
		return 10;
	}

	const vo::EssentialMatrixResult essential_result =
		vo::estimateEssentialMatrixRansac(
			undistorted_correspondences,
			ratio_matches,
			calibration.camera_matrix,
			kEssentialRansacThreshold,
			kRansacConfidence
		);
	if (essential_result.essential_matrix.empty())
	{
		std::cerr
			<< "Failed to estimate the essential matrix.\n";
		return 15;
	}
	const vo::RelativePoseResult pose_result =
		vo::recoverRelativePose(
			essential_result,
			undistorted_correspondences,
			ratio_matches,
			calibration.camera_matrix
		);

	if (!pose_result.valid)
	{
		std::cerr
			<< "Failed to recover relative camera pose.\n";
		return 16;
	}
	// ransac 内点率
	const double essential_inlier_rate =
		100.0
		* static_cast<double>(
			essential_result.inlier_matches.size()
			)
		/ static_cast<double>(ratio_matches.size());
	// 正深度内点占本质矩阵内点的比例
	const double cheirality_inlier_rate =
		static_cast<double>(pose_result.cheirality_inlier_count)
		/ static_cast<double>(essential_result.inlier_matches.size());

	const bool pose_is_reliable =
		pose_result.cheirality_inlier_count >= kMinCheiralityInliers
		&& cheirality_inlier_rate >= kMinCheiralityInlierRate;
	if (!pose_is_reliable)
	{
		std::cerr
			<< "Recovered pose is unreliable:\n"
			<< "  Cheirality inliers: "
			<< pose_result.cheirality_inlier_count
			<< " / "
			<< essential_result.inlier_matches.size()
			<< '\n'
			<< "  Cheirality inlier rate: "
			<< cheirality_inlier_rate * 100.0
			<< "%\n";

		return 17;
	}

	return 0;
}
