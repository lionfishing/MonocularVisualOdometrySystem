#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

void printImageInfo(const std::string& name, const cv::Mat& image);

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: project_3_vo <image_path>\n";
        return 1;
    }
    const std::string image_path = argv[1];
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cerr << "Failed to load image: " << image_path << '\n';
        return 2;
    }

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    const cv::Rect roi_rect(image.cols / 4, image.rows / 4, image.cols / 2, image.rows / 2);
    const cv::Mat roi = image(roi_rect);
    const cv::Mat roi_copy = roi.clone();

    printImageInfo("ROI image", roi);
    printImageInfo("ROI copy", roi_copy);

    cv::imshow("Input image", image);
    cv::imshow("Grayscale image", gray_image);
    cv::imshow("ROI image", roi);
    cv::waitKey(0);

    return 0;
}

void printImageInfo(
    const std::string& name,
    const cv::Mat& image)
{
    std::cout << '\n' << name << ":\n";
    std::cout << "Rows: " << image.rows << '\n';
    std::cout << "Cols: " << image.cols << '\n';
    std::cout << "Type: " << image.type() << '\n';
    std::cout << "Channels: " << image.channels() << '\n';
    std::cout << "Element size: "
        << image.elemSize() << " bytes\n";
    std::cout << "Valid row bytes: "
        << image.cols * image.elemSize() << " bytes\n";
    std::cout << "Step: " << image.step << " bytes\n";
    std::cout << "Continuous: "
        << std::boolalpha
        << image.isContinuous() << '\n';
}