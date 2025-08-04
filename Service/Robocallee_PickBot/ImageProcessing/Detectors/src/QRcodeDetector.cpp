#include "QRcodeDetector.hpp"

using namespace std;
using namespace cv;
using namespace OB;

QRcodeDetector::QRcodeDetector()
{
    qrDecoder_ = new cv::QRCodeDetector;
}

QRcodeDetector::~QRcodeDetector()
{
    delete qrDecoder_;
    qrDecoder_ = nullptr;
}

bool QRcodeDetector::initDetector()
{
    return true;
}

bool QRcodeDetector::FindObject(cv::Mat& image, std::vector<object2d>& objects) 
{
    if(image.empty()) return false;

    if(!objects.empty()) return false;

    try
    {
        vector<std::string> decoded_info;
        vector<cv::Point> points;
        if(!qrDecoder_->detectAndDecodeMulti(image, decoded_info, points)) return false;

        int Size = decoded_info.size();
        for(int i = 0 ; i < Size; ++i)
        {
            object2d object;
            object.objectID = decoded_info[i];
            object.points.push_back(points[i]);
            object.points.push_back(points[i+1]);
            object.points.push_back(points[i+2]);
            object.points.push_back(points[i+3]);
            objects.push_back(object);
        }

        if(objects.empty()) return false;
    
        return true;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[OpenCV 예외] detectAndDecodeMulti: " << e.what() << std::endl;
        return false;
    }
}

bool QRcodeDetector::EstimatePose
(
    const cv::Mat& image,                       // 입력 이미지 (Grayscale or Color)
    const cv::Mat& cameraMatrix,                // 내부 파라미터 K
    const cv::Mat& distCoeffs,                  // 왜곡 계수 D
    const double targetsize,                    // mm
    std::vector<cv::Mat>& rvecs,                // 출력: 회전 벡터들
    std::vector<cv::Mat>& tvecs                 // 출력: 위치 벡터들
)
{
    
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // 마커 감지
    //cv::aruco::detectMarkers(image, dictionary_, markerCorners, markerIds);

    if (markerIds.empty())
        return false;

    // Pose 추정
    try
    {
        //cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength_, cameraMatrix, distCoeffs, rvecs, tvecs);
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[OpenCV 예외] ArUcoMakerDetector::EstimatePose : " << e.what() << std::endl;
        return false;
    }
    

    return true;
}