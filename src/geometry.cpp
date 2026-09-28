#include "geometry.h"

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

}// namespace vo