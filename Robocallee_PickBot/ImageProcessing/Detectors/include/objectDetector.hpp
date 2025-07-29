#pragma once

#include <memory>
#include "opencv2/opencv.hpp"

namespace OB
{
    typedef struct object2d
    {
        cv::Point CenterPt;
        std::string objectID;
        std::vector<cv::Point> points;
    } object2d;

    typedef struct object3d
    {
        cv::Point3d CenterPt;
        std::string objectID;
        std::vector<cv::Point3d> points;
    } object3d;

    class objectDetector
    {
    protected:

    public:
        using s_ptr = std::shared_ptr<objectDetector>;
        
        objectDetector(){}
        virtual ~objectDetector(){}

        virtual bool initDetector() = 0;

        virtual bool FindObject(cv::Mat& image, std::vector<object2d>& objects) = 0;

        virtual bool EstimatePose(
            const cv::Mat& image,                       // 입력 이미지 (Grayscale or Color)
            const cv::Mat& cameraMatrix,                // 내부 파라미터 K
            const cv::Mat& distCoeffs,                  // 왜곡 계수 D
            const double targetsize,                    // mm
            std::vector<cv::Mat>& rvecs,                // 출력: 회전 벡터들
            std::vector<cv::Mat>& tvecs                 // 출력: 위치 벡터들
        ) = 0;
    };
}
