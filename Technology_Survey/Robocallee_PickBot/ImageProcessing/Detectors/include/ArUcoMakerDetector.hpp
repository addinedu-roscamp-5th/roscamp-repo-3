#pragma once

#include "objectDetector.hpp"
#include <opencv2/aruco.hpp>

namespace OB
{
    class ArUcoMakerDetector : public OB::objectDetector
    {
    private:
        cv::Ptr<cv::aruco::Dictionary>      dictionary_;


    public:
        ArUcoMakerDetector();
        virtual ~ArUcoMakerDetector();

        virtual bool initDetector() override;

        virtual bool FindObject(cv::Mat& image, std::vector<object2d>& objects) override;

        virtual bool EstimatePose(
            const cv::Mat& image,                       // 입력 이미지 (Grayscale or Color)
            const cv::Mat& cameraMatrix,                // 내부 파라미터 K
            const cv::Mat& distCoeffs,                  // 왜곡 계수 D
            const double targetsize,                    // mm
            std::vector<cv::Mat>& rvecs,                // 출력: 회전 벡터들
            std::vector<cv::Mat>& tvecs                 // 출력: 위치 벡터들
            ) override;
    };
}