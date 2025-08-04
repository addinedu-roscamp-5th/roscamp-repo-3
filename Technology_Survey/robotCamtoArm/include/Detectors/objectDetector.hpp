#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

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

        virtual bool TransformCameraPose(
            std::vector<cv::Point>& point2d,        // mm
            std::vector<cv::Point3d>& point3d,      // mm
            const cv::Mat& cameraMatrix,
            const cv::Mat& distCoeffs,
            const double depth,
            const bool useUnDist = false
        )
        {
            if(point2d.empty() || cameraMatrix.empty() || distCoeffs.empty()) return false;
            
            point3d.clear();
            point3d.reserve(point2d.size());
            
            if(useUnDist)
            {
                std::vector<cv::Point2f> point2f(point2d.begin(), point2d.end());
                std::vector<cv::Point2f> undistorted2f;

                // 왜곡 보정 → 내부 파라미터까지 적용됨 → z=1 평면상의 좌표
                cv::undistortPoints(point2f, undistorted2f, cameraMatrix, distCoeffs);

                for (const auto& pt : undistorted2f)
                {
                    // 동차 좌표계로 확장
                    cv::Point3d vec3D(pt.x, pt.y, 1.0);

                    // 단위 벡터로 정규화
                    double norm = cv::norm(vec3D);
                    cv::Point3d unitVec = vec3D * (1.0 / norm) * depth;
                    
                    point3d.emplace_back(unitVec); // 방향 성분만 있는 방향 벡터
                }
            }

            else
            {
                cv::Mat invk = cameraMatrix.inv();
                for (const auto& pt : point2d)
                {
                    cv::Mat uv = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1.0);
                    cv::Mat dir = cameraMatrix.inv() * uv;
                    
                    double x = dir.at<double>(0, 0) / dir.at<double>(2, 0);
                    double y = dir.at<double>(1, 0) / dir.at<double>(2, 0);

                    cv::Point3d vec3D(x, y, 1.0);

                    // 단위 벡터로 정규화
                    double norm = cv::norm(vec3D);
                    cv::Point3d unitVec = vec3D * (1.0 / norm) * depth;
                    
                    point3d.emplace_back(unitVec);  // 방향 성분만 있는 방향 벡터
                }
            }

            return true;
        }

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
