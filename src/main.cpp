#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

void printImageInfo(const std::string& name, const cv::Mat& image);

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: project_3_vo <image_path>\n";
        return 1;
    }
    
    const cv::Mat image2 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    const cv::Mat image3 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (image2.empty())
    {
        std::cerr << "Failed to load image: " << argv[1] << '\n';
        return 2;
    }
    if (image3.empty())
    {
        std::cerr << "Failed to load image: " << argv[2] << '\n';
        return 2;
    }
    const cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);

    std::vector<cv::KeyPoint> keypoints2;
    std::vector<cv::KeyPoint> keypoints3;

    cv::Mat descriptors2;
    cv::Mat descriptors3;

    orb->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    orb->detectAndCompute(image3, cv::noArray(), keypoints3, descriptors3);

    if (descriptors2.empty())
    {
        std::cerr << "Failed to compute descriptors for image 1.\n";
        return 4;
    }

    if (descriptors3.empty())
    {
        std::cerr << "Failed to compute descriptors for image 2.\n";
        return 5;
    }

    cv::Mat keypoints_view2;
    cv::Mat keypoints_view3;

    cv::drawKeypoints(image2, keypoints2, keypoints_view2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(image3, keypoints3, keypoints_view3, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::imshow("Image 2 keypoints", keypoints_view2);
    cv::imshow("Image 3 keypoints", keypoints_view3);
    cv::waitKey(0);
    
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