#pragma once

#include "Integrated.hpp"
#include "opencv2/opencv.hpp"

namespace geometry
{
    class Geometry
    {
        public:
            // ----------- 변환 함수 -----------
            template<typename PointT>
            static cv::Mat pointToMat(const PointT& pt);

            template<typename PointT>
            static PointT matToPoint(const cv::Mat& mat);

        
            Geometry() = default;
            ~Geometry() = default;

            // ----------- 기하학 연산 함수 -----------
            static bool toHomogeneous(cv::Mat& in , cv::Mat& out);

            static bool formHomogeneous(cv::Mat& in , cv::Mat& out);

            static bool homogeneousInverse(const cv::Mat& T, cv::Mat& invT);

            static bool composeTransform(const cv::Mat& rvec, const cv::Mat& tvec, cv::Mat& T_se3);

            static bool decomposeTransform(const cv::Mat& T_se3 , cv::Mat& rvec , cv::Mat& tvec);
            
            static bool transformPoint(const cv::Mat& R, const cv::Mat& t, const cv::Point3d& p);

            static bool transformCameraPose
            (
                cv::Point point2d, // mm
                cv::Point3d point3d, // mm
                const cv::Mat& cameraMatrix,
                const cv::Mat& distCoeffs,
                const bool useUnDist = false
            );

            static bool estimateRigidTransformSVD(const std::vector<cv::Point3d>& P1, const std::vector<cv::Point3d>& P2, cv::Mat& R, cv::Mat& t);
    };

    // -------------------------
    // Point -> Mat 변환
    // -------------------------

    template<typename PointT>
    cv::Mat Geometry::pointToMat(const PointT& pt)
    {
        // 2D Point
        if constexpr (std::is_same<PointT, cv::Point2f>::value || std::is_same<PointT, cv::Point2d>::value ||
                    std::is_same<PointT, cv::Point2i>::value || std::is_same<PointT, cv::Point2l>::value)
                    {
            using ScalarT = decltype(pt.x);
            return (cv::Mat_<ScalarT>(2, 1) << pt.x, pt.y);
        }

        // 3D Point
        else if constexpr (std::is_same<PointT, cv::Point3f>::value || std::is_same<PointT, cv::Point3d>::value ||
                        std::is_same<PointT, cv::Point3i>::value || std::is_same<PointT, cv::Point3l>::value)
                        {
            using ScalarT = decltype(pt.x);
            return (cv::Mat_<ScalarT>(3, 1) << pt.x, pt.y, pt.z);
        }

        else
        {
            static_assert(!std::is_same<PointT, PointT>::value, "지원하지 않는 Point 타입입니다.");
        }
    }

    // -------------------------
    // Mat -> Point 변환
    // -------------------------
    template<typename PointT>
    PointT Geometry::matToPoint(const cv::Mat& mat)
    {
        // Mat 크기 검증
        CV_Assert(mat.total() == 2 || mat.total() == 3);
        CV_Assert(mat.channels() == 1);

        // 2D Point
        if constexpr (
            std::is_same_v<PointT, cv::Point2f> || std::is_same_v<PointT, cv::Point2d> ||
            std::is_same_v<PointT, cv::Point2i> || std::is_same_v<PointT, cv::Point2l>
        ) {
            CV_Assert(mat.total() == 2);
            return PointT(
                mat.at<typename PointT::value_type>(0, 0),
                mat.at<typename PointT::value_type>(1, 0)
            );
        }
        // 3D Point
        else if constexpr (
            std::is_same_v<PointT, cv::Point3f> || std::is_same_v<PointT, cv::Point3d> ||
            std::is_same_v<PointT, cv::Point3i> || std::is_same_v<PointT, cv::Point3l>
        ) {
            CV_Assert(mat.total() == 3);
            return PointT(
                mat.at<typename PointT::value_type>(0, 0),
                mat.at<typename PointT::value_type>(1, 0),
                mat.at<typename PointT::value_type>(2, 0)
            );
        }
        else {
            static_assert(!std::is_same<PointT, PointT>::value,
                          "지원하지 않는 Point 타입입니다.");
        }
    }
}
