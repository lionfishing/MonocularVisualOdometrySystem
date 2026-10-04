#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace vo
{

	struct CameraCalibration
	{
		std::string camera_model;
		std::string distortion_model;

		cv::Size image_size;
		cv::Mat camera_matrix;
		cv::Mat distortion_coefficients;

		bool isValid() const;
	};

	bool loadCameraCalibration(
		const std::string& file_path,
		CameraCalibration& calibration
	);
}  // namespace vo
