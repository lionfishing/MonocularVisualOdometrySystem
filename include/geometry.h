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
PointCorrespondences buildPointCorrespondences(
	const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2,
	const std::vector<cv::DMatch>& matches
);

}// namespace vo