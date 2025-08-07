#pragma once

#include "Integrated.hpp"
#include "Commondefine.hpp"
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

            static bool decomposeTransform(const cv::Mat& T_se3 , cv::Mat& rvec , cv::Mat& tvec, bool rodrigues = false);
            
            static bool transformPoint(const cv::Mat& R, const cv::Mat& t, const cv::Point3d& p);

            static double transformCameraPose
            (
                cv::Point point2d, // mm
                cv::Point3d& point3d, // mm
                const cv::Mat& cameraMatrix,
                const cv::Mat& distCoeffs,
                const bool useNorm = false,
                const bool useUnDist = false
            );

            static bool SkewSymmetricMatrix(const cv::Vec3d& v, cv::Mat& m);

            static bool computeEssentialMatrixFromSE3(const cv::Mat& R, const cv::Mat& t,cv::Mat& E);

            static bool estimateRigidTransformSVD(const std::vector<cv::Point3d>& P1, const std::vector<cv::Point3d>& P2, cv::Mat& R, cv::Mat& t);
            
            static bool dh_to_transform(const double a, const  double alpha, const double d, const double theta , cv::Mat& T);

            static bool compute_forward_kinematics(const std::vector<double>& joint_angles, const std::vector<std::array<double, 4>> DH, cv::Mat& FK);
    };

    // -------------------------
    // Point -> Mat 변환
    // -------------------------

    template<typename PointT>
    cv::Mat Geometry::pointToMat(const PointT& pt)
    {
        // 2D Point
        if constexpr (std::is_same<PointT, cv::Point2f>::value || std::is_same<PointT, cv::Point2d>::value ||
                    std::is_same<PointT, cv::Point2i>::value || std::is_same<PointT, cv::Point>::value)
                    {
            using ScalarT = decltype(pt.x);
            return (cv::Mat_<ScalarT>(2, 1) << pt.x, pt.y);
        }

        // 3D Point
        else if constexpr (std::is_same<PointT, cv::Point3f>::value || std::is_same<PointT, cv::Point3d>::value ||
                        std::is_same<PointT, cv::Point3i>::value)
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
    using ValueT = typename PointT::value_type;

    // 1. 다채널 Vec 기반: (1×1) 행렬 + 2채널 or 3채널
    if (mat.rows == 1 && mat.cols == 1)
    {
        if constexpr (
            std::is_same_v<PointT, cv::Point2f> || std::is_same_v<PointT, cv::Point2d> ||
            std::is_same_v<PointT, cv::Point2i>)
        {
            CV_Assert(mat.channels() == 2);
            cv::Vec<ValueT, 2> vec = mat.at<cv::Vec<ValueT, 2>>(0, 0);
            return PointT(vec[0], vec[1]);
        }
        else if constexpr (
            std::is_same_v<PointT, cv::Point3f> || std::is_same_v<PointT, cv::Point3d> ||
            std::is_same_v<PointT, cv::Point3i>)
        {
            CV_Assert(mat.channels() == 3);
            cv::Vec<ValueT, 3> vec = mat.at<cv::Vec<ValueT, 3>>(0, 0);
            return PointT(vec[0], vec[1], vec[2]);
        }
    }

    // 2. 단채널 일반 행렬: (2×1) 또는 (3×1)
    CV_Assert(mat.channels() == 1);
    CV_Assert(mat.cols == 1);  // 열이 반드시 1개

    if constexpr (
        std::is_same_v<PointT, cv::Point2f> || std::is_same_v<PointT, cv::Point2d> ||
        std::is_same_v<PointT, cv::Point2i>)
    {
        CV_Assert(mat.rows == 2);
        return PointT(
            mat.at<ValueT>(0, 0),
            mat.at<ValueT>(1, 0)
        );
    }
    else if constexpr (
        std::is_same_v<PointT, cv::Point3f> || std::is_same_v<PointT, cv::Point3d> ||
        std::is_same_v<PointT, cv::Point3i>)
    {
        CV_Assert(mat.rows == 3);
        return PointT(
            mat.at<ValueT>(0, 0),
            mat.at<ValueT>(1, 0),
            mat.at<ValueT>(2, 0)
        );
    }
    else
    {
        static_assert(!std::is_same<PointT, PointT>::value,
                      "지원하지 않는 Point 타입입니다.");
    }
}
}
