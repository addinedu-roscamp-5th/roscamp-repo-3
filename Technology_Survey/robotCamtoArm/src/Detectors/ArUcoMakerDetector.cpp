#include "../../include/Detectors/ArUcoMakerDetector.hpp"

using namespace std;
using namespace cv;
using namespace aruco;
using namespace OB;

ArUcoMakerDetector::ArUcoMakerDetector()
{
}

ArUcoMakerDetector::~ArUcoMakerDetector()
{
}

bool ArUcoMakerDetector::initDetector()
{
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50));
    
    return true;
}

bool ArUcoMakerDetector::FindObject(cv::Mat& image, std::vector<object2d>& objects) 
{
    if(image.empty()) return false;

    if(!objects.empty()) return false;

    try
    {
        vector<int> markerIds;
        vector<vector<cv::Point2f>> markerCorners;
        detectMarkers(image, dictionary_, markerCorners, markerIds);

        if(markerIds.empty()) return false;

        int Size = markerIds.size();
        
        for(int i = 0; i < Size; ++i)
        {
            object2d object;
            object.objectID = to_string(markerIds[i]);
          
            cv::Point2f center(0.0, 0.0);
            for (const auto& pt : markerCorners[i])
            {
                object.points.push_back(pt);
                center += pt;
            }
            center *= 0.25f;
            object.CenterPt = center;

            objects.push_back(object);
        }

        if(objects.empty()) return false;
    
        return true;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[OpenCV 예외] ArUcoMakerDetector::detectMarkers : " << e.what() << std::endl;
        return false;
    }
}

// 마커 pose 추정 함수
bool ArUcoMakerDetector::EstimatePose
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
    cv::aruco::detectMarkers(image, dictionary_, markerCorners, markerIds);

    if (markerIds.empty()) return false;

    try
    {
        double mtargetsize = targetsize;
        std::vector<cv::Vec3d> rvec;
        std::vector<cv::Vec3d> tvec;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, mtargetsize, cameraMatrix, distCoeffs, rvec, tvec);

        for (size_t i = 0; i < rvec.size(); ++i)
        {
            rvecs.push_back(cv::Mat(rvec[i]));
            tvecs.push_back(cv::Mat(tvec[i]));
        }
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "[OpenCV 예외] ArUcoMakerDetector::EstimatePose : " << e.what() << std::endl;
        return false;
    }
    

    return true;
}