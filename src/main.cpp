#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: project_3_vo <image_path> \n";
        return 1;
    }

    const std::string image_path = argv[1];

    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cerr << "Failed to load image: " << image_path << '\n';
        return 2;
    }


    std::cout << "OpenCV version: " << CV_VERSION << '\n';
    std::cout << "Image path: " << image_path << '\n';
    std::cout << "Image size: " << image.cols << " x " << image.rows << '\n';
    std::cout << "Channels: " << image.channels() << '\n';

    cv::imshow("Input image", image);
    cv::waitKey(0);

    return 0;
}