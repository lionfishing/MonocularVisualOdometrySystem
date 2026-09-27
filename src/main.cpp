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
        std::cerr << "Usage: project_3_vo <image1_path> <image2_path>\n";
        return 1;
    }
    
    const cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    const cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (image1.empty())
    {
        std::cerr << "Failed to load image: " << argv[1] << '\n';
        return 2;
    }
    if (image2.empty())
    {
        std::cerr << "Failed to load image: " << argv[1] << '\n';
        return 3;
    }
    const cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;

    cv::Mat descriptors1;
    cv::Mat descriptors2;

    orb->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

    if (descriptors1.empty())
    {
        std::cerr << "Failed to compute descriptors for image 1.\n";
        return 4;
    }

    if (descriptors2.empty())
    {
        std::cerr << "Failed to compute descriptors for image 1.\n";
        return 5;
    }

    cv::Mat keypoints_view1;
    cv::Mat keypoints_view2;

    cv::drawKeypoints(
        image1, 
        keypoints1, 
        keypoints_view1, 
        cv::Scalar::all(-1), 
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(
        image2, keypoints2, 
        keypoints_view2, 
        cv::Scalar::all(-1), 
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    std::cout << "Image 1 keypoints: "
        << keypoints1.size() << '\n';

    std::cout << "Image 2 keypoints: "
        << keypoints2.size() << '\n';

    std::cout << "Descriptors 1: "
        << descriptors1.rows << " x "
        << descriptors1.cols << '\n';

    std::cout << "Descriptors 2: "
        << descriptors2.rows << " x "
        << descriptors2.cols << '\n';

    std::cout << "Descriptor type: "
        << descriptors1.type() << '\n';

    std::cout << "Bytes per descriptor: "
        << descriptors1.cols * descriptors1.elemSize()
        << '\n';


    cv::imshow("Image 1 keypoints", keypoints_view1);
    cv::imshow("Image 2 keypoints", keypoints_view2);
    cv::waitKey(0);

    return 0;
}
