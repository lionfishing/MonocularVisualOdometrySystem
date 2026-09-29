#include <iostream>
#include <vector>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

//内角点数量
constexpr int kBoardColumns = 9;
constexpr int kBoardRows = 6;
//实际一格的宽度
constexpr float kSquareSizeMillimeters = 23.5F;

namespace fs = std::filesystem;

//生成棋盘格坐标
std::vector<cv::Point3f> createBoardObjectPoints(
    const cv::Size& board_size,
    float square_size)
{
    std::vector<cv::Point3f> object_points;

    object_points.reserve(
        board_size.width * board_size.height
    );

    for (int row = 0; row < board_size.height; ++row)
    {
        for (int column = 0;
            column < board_size.width;
            ++column)
        {
            object_points.emplace_back(
                column * square_size,
                row * square_size,
                0.0F
            );
        }
    }

    return object_points;
}

bool isSupportedImageFile(const fs::path& path)
{
    std::string extension = path.extension().string();

    std::transform(
        extension.begin(),
        extension.end(),
        extension.begin(),
        [](unsigned char character)
        {
            return static_cast<char>(
                std::tolower(character)
                );
        }
    );

    return extension == ".jpg"
        || extension == ".jpeg"
        || extension == ".png";
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: camera_calibrate "
            << "<calibration_image_directory>\n";
        return 1;
    }
    //转为路径对象
    const fs::path calibration_directory(argv[1]);
    //验证路径是否存在/路径是否为目录
    if (!fs::exists(calibration_directory)
        || !fs::is_directory(calibration_directory))
    {
        std::cerr
            << "Invalid calibration directory: "
            << calibration_directory
            << '\n';
        return 2;
    }
    //保存图片路径
    std::vector<fs::path> image_paths;
    //枚举图片文件
    for (const fs::directory_entry& entry :
        fs::directory_iterator(calibration_directory))
    {
        //是普通文件且是图片文件
        if (entry.is_regular_file()
            && isSupportedImageFile(entry.path()))
        {
            image_paths.push_back(entry.path());
        }
    }

    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.empty())
    {
        std::cerr
            << "No calibration images found in: "
            << calibration_directory
            << '\n';
        return 3;
    }
    //棋盘格检测参数 顺序为（列数，行数）
    const cv::Size board_size(
        kBoardColumns,
        kBoardRows
    );
    //根据局部亮度自适应区分黑白区域 
    //检测前对图像亮度进行归一化
    const int detection_flags =
        cv::CALIB_CB_ADAPTIVE_THRESH
        | cv::CALIB_CB_NORMALIZE_IMAGE;

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<fs::path> accepted_image_paths;

    cv::Size expected_image_size;

    for (const fs::path& image_path : image_paths)
    {
        cv::Mat image =
            cv::imread(
                image_path.string(),
                cv::IMREAD_COLOR
            );

        if (image.empty())
        {
            std::cout
                << "[READ FAILED] "
                << image_path.filename()
                << '\n';
            continue;
        }

        if (expected_image_size.width == 0)
        {
            expected_image_size = image.size();
        }
        //每张图片像素必须相同
        else if (image.size() != expected_image_size)
        {
            std::cout
                << "[SIZE MISMATCH] "
                << image_path.filename()
                << ": "
                << image.cols
                << " x "
                << image.rows
                << '\n';
            continue;
        }

        cv::Mat grayscale;
        cv::cvtColor(
            image,
            grayscale,
            cv::COLOR_BGR2GRAY
        );

        std::vector<cv::Point2f> corners;

        const bool found =
            cv::findChessboardCorners(
                grayscale,
                board_size,
                corners,
                detection_flags
            );

        if (!found)
        {
            std::cout
                << "[NOT FOUND] "
                << image_path.filename()
                << '\n';
            continue;
        }
        //亚像素优化
        cv::cornerSubPix(
            grayscale,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(
                cv::TermCriteria::EPS
                | cv::TermCriteria::COUNT,
                30,//最大迭代次数
                0.001//精度阈值
            )
        );

        image_points.push_back(corners);
        accepted_image_paths.push_back(image_path);

        std::cout
            << "[OK] "
            << image_path.filename()
            << ": "
            << corners.size()
            << " corners\n"; 

    }

    const std::size_t accepted_count =
        image_points.size();

    const std::size_t rejected_count =
        image_paths.size() - accepted_count;

    std::cout
        << "\nCalibration image validation summary:\n"
        << "  Total: " << image_paths.size() << '\n'
        << "  Accepted: " << accepted_count << '\n'
        << "  Rejected: " << rejected_count << '\n'
        << "  Image size: "
        << expected_image_size.width
        << " x "
        << expected_image_size.height
        << '\n';

    if (accepted_count < 10)
    {
        std::cerr
            << "Not enough valid calibration images.\n";
        return 5;
    }

    const std::vector<cv::Point3f> board_object_points =
    createBoardObjectPoints(
        board_size,
        kSquareSizeMillimeters
    );

//创建image_points.size() 份 board_object_points。
const std::vector<std::vector<cv::Point3f>> object_points(
    image_points.size(),
    board_object_points
);

//内参矩阵 待求解
cv::Mat camera_matrix =
    cv::Mat::eye(3, 3, CV_64F);
//畸变矩阵 待求解
cv::Mat distortion_coefficients;
//旋转
std::vector<cv::Mat> rotation_vectors;
//位移
std::vector<cv::Mat> translation_vectors;
//RMS重投影误差
const double rms_reprojection_error =
    cv::calibrateCamera(
        object_points,
        image_points,
        expected_image_size,
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors
    );

std::cout
    << "\nCalibration result:\n"
    << "RMS reprojection error: "
    << rms_reprojection_error
    << '\n';

std::cout
    << "Camera matrix:\n"
    << camera_matrix
    << '\n';

std::cout
    << "Distortion coefficients:\n"
    << distortion_coefficients.t()
    << '\n';

std::cout
    << "Estimated camera poses: "
    << rotation_vectors.size()
    << '\n';

    return 0;
}