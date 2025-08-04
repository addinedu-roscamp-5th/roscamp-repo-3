#pragma once

#include "objectDetector.hpp"

namespace OB
{
    class QRcodeDetector : public OB::objectDetector
    {
    private:
        cv::QRCodeDetector* qrDecoder_;



    public:
        QRcodeDetector();
        ~QRcodeDetector();

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
 
