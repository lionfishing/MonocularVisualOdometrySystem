#include "camera_calibration.h"

namespace vo
{

    bool CameraCalibration::isValid() const
    {
        return camera_model == "pinhole"
            && distortion_model == "opencv"
            && image_size.width > 0
            && image_size.height > 0
            && camera_matrix.rows == 3
            && camera_matrix.cols == 3
            && camera_matrix.type() == CV_64F
            && distortion_coefficients.total() == 5
            && distortion_coefficients.type() == CV_64F;
    }

    bool loadCameraCalibration(
        const std::string& file_path,
        CameraCalibration& calibration)
    {
        cv::FileStorage storage(
            file_path,
            cv::FileStorage::READ
        );

        if (!storage.isOpened())
        {
            return false;
        }
        //先写入临时对象，避免留下“半有效”对象
        CameraCalibration loaded_calibration;

        int image_width = 0;
        int image_height = 0;

        storage["camera_model"]
            >> loaded_calibration.camera_model;

        storage["distortion_model"]
            >> loaded_calibration.distortion_model;

        storage["image_width"] >> image_width;
        storage["image_height"] >> image_height;

        storage["camera_matrix"]
            >> loaded_calibration.camera_matrix;

        storage["distortion_coefficients"]
            >> loaded_calibration.distortion_coefficients;

        storage.release();

        loaded_calibration.image_size =
            cv::Size(image_width, image_height);

        if (!loaded_calibration.isValid())
        {
            return false;
        }

        calibration = loaded_calibration;
        return true;
    }

}