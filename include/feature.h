#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace vo
{

	struct FeatureSet
	{
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
	};
	FeatureSet extractOrbFeatures(
		const cv::Mat& image,
		int max_features
	);
}