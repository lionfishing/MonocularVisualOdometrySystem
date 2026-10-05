#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <system_error>

#include <opencv2/imgcodecs.hpp>

#include "camera_calibration.h"
#include "visual_odometry.h"

namespace filesystem = std::filesystem;

bool isSupportedImage(const filesystem::path& path)
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

void writeTrajectoryRow(
    std::ostream& output,
    const filesystem::path& image_path,
    const char* status,
    const vo::VisualOdometryResult& result
)
{
    output
        << '"'
        << image_path.filename().string()
        << '"'
        << ','
        << status
        << ','
        << result.match_count
        << ','
        << result.essential_inlier_count
        << ','
        << result.cheirality_inlier_count;

    if (result.position_world.empty())
    {
        output << ",,,\n";
        return;
    }

    output
        << ','
        << result.position_world.at<double>(0, 0)
        << ','
        << result.position_world.at<double>(1, 0)
        << ','
        << result.position_world.at<double>(2, 0)
        << '\n';
}

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cerr
            << "Usage: project_3_vo "
            << "<sequence_directory> "
            << "<calibration_file> "
            << "<trajectory_csv>\n";

        return 1;
    }

    const filesystem::path sequence_directory = argv[1];
    const filesystem::path trajectory_path = argv[3];

    if (!filesystem::exists(sequence_directory)
        || !filesystem::is_directory(sequence_directory))
    {
        std::cerr
            << "Invalid sequence directory: "
            << sequence_directory
            << '\n';

        return 2;
    }

    vo::CameraCalibration calibration;

    if (!vo::loadCameraCalibration(
        argv[2],
        calibration
    ))
    {
        std::cerr
            << "Failed to load camera calibration: "
            << argv[2]
            << '\n';

        return 3;
    }

    std::vector<filesystem::path> image_paths;

    for (const filesystem::directory_entry& entry
        : filesystem::directory_iterator(sequence_directory))
    {
        if (entry.is_regular_file()
            && isSupportedImage(entry.path()))
        {
            image_paths.push_back(entry.path());
        }
    }

    std::sort(image_paths.begin(), image_paths.end());

    if (image_paths.size() < 2)
    {
        std::cerr
            << "At least two sequence images are required.\n";

        return 4;
    }

    if (!trajectory_path.parent_path().empty())
    {
        std::error_code error;

        filesystem::create_directories(
            trajectory_path.parent_path(),
            error
        );

        if (error)
        {
            std::cerr
                << "Failed to create output directory: "
                << trajectory_path.parent_path()
                << '\n';

            return 5;
        }
    }

    std::ofstream trajectory_file(trajectory_path);

    if (!trajectory_file)
    {
        std::cerr
            << "Failed to open trajectory file: "
            << trajectory_path
            << '\n';

        return 6;
    }

    trajectory_file << std::setprecision(10);

    trajectory_file
        << "frame,status,matches,"
        << "essential_inliers,cheirality_inliers,"
        << "x,y,z\n";
    vo::MonocularVisualOdometry visual_odometry(
        calibration
    );

    bool sequence_initialized = false;

    for (const filesystem::path& image_path : image_paths)
    {
        const cv::Mat image = cv::imread(
            image_path.string(),
            cv::IMREAD_GRAYSCALE
        );

        if (image.empty())
        {
            std::cerr
                << "[SKIP] Failed to load: "
                << image_path.filename()
                << '\n';

            const vo::VisualOdometryResult failed_result;

            writeTrajectoryRow(
                trajectory_file,
                image_path,
                "LOAD_FAILED",
                failed_result
            );

            continue;
        }

        const vo::VisualOdometryResult result =
            visual_odometry.processFrame(image);

        if (!sequence_initialized)
        {
            if (result.initialized)
            {
                sequence_initialized = true;

                std::cout
                    << "[INIT] "
                    << image_path.filename()
                    << " position = (0, 0, 0)\n";

                writeTrajectoryRow(
                    trajectory_file,
                    image_path,
                    "INIT",
                    result
                );
            }
            else
            {
                std::cerr
                    << "[SKIP] Failed to initialize from: "
                    << image_path.filename()
                    << '\n';

                writeTrajectoryRow(
                    trajectory_file,
                    image_path,
                    "SKIP",
                    result
                );
            }

            continue;
        }

        if (!result.pose_updated)
        {
            std::cout
                << "[SKIP] "
                << image_path.filename()
                << " matches="
                << result.match_count
                << " essential_inliers="
                << result.essential_inlier_count
                << " cheirality_inliers="
                << result.cheirality_inlier_count
                << '\n';

            writeTrajectoryRow(
                trajectory_file,
                image_path,
                "SKIP",
                result
            );

            continue;
        }

        std::cout
            << "[OK] "
            << image_path.filename()
            << " matches="
            << result.match_count
            << " essential_inliers="
            << result.essential_inlier_count
            << " cheirality_inliers="
            << result.cheirality_inlier_count
            << " position=("
            << result.position_world.at<double>(0, 0)
            << ", "
            << result.position_world.at<double>(1, 0)
            << ", "
            << result.position_world.at<double>(2, 0)
            << ")\n";

        writeTrajectoryRow(
            trajectory_file,
            image_path,
            "OK",
            result
        );
    }

    trajectory_file.close();

    if (!trajectory_file)
    {
        std::cerr
            << "Failed while writing trajectory file: "
            << trajectory_path
            << '\n';

        return 7;
    }

    std::cout
        << "Trajectory saved to: "
        << trajectory_path
        << '\n';

    return 0;
}
