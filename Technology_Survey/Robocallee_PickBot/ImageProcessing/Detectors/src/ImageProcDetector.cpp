#include "ImageProcDetector.hpp"

using namespace OB;
using namespace cv;

ImageProcDetector::ImageProcDetector()
{

}

ImageProcDetector::~ImageProcDetector()
{
    
}

bool ImageProcDetector::initDetector()
{
    return true;
}

bool ImageProcDetector::FindObject(cv::Mat& image, std::vector<object2d>& objects)
{
    if(image.empty()) return false;

    Mat gray;
    cv::cvtColor(image, gray, COLOR_BGR2GRAY);

    int th = 128;

    return true;
}

bool ImageProcDetector::EstimatePose(
    const cv::Mat& image,                       // 입력 이미지 (Grayscale or Color)
    const cv::Mat& cameraMatrix,                // 내부 파라미터 K
    const cv::Mat& distCoeffs,                  // 왜곡 계수 D
    const double targetsize,                    // mm
    std::vector<cv::Mat>& rvecs,                // 출력: 회전 벡터들
    std::vector<cv::Mat>& tvecs                 // 출력: 위치 벡터들
)
{
    return true;
}