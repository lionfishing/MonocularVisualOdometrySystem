#include "feature.h"

namespace vo
{
	FeatureSet extractOrbFeatures(
		const cv::Mat& image,
		int max_features)
	{
        FeatureSet features;

        const cv::Ptr<cv::ORB> orb =
            cv::ORB::create(max_features);

        orb->detectAndCompute(
            image,
            cv::noArray(),
            features.keypoints,
            features.descriptors
        );

        return features;
	}
}