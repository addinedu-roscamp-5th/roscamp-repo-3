#include "DepthEstimate.hpp"


using namespace depth;
using namespace Integrated;
using namespace cv;
using namespace OB;
using namespace std;
using namespace geometry;

DepthEstimate::DepthEstimate(
    Integrated::w_ptr<OB::objectDetector> detector,
    Integrated::w_ptr<Calib::Calibrator> calib
    )
    : detector_(detector), calib_(calib)
{

}

DepthEstimate::~DepthEstimate()
{

}

double DepthEstimate::MonoDepthEstimate
(
    vec<cv::Point>& left,
    vec<cv::Point>& right,
    vec<cv::Point3d>& pointcloud,
    double baseline
)
{
    const double Err = -1;

    if(left.empty() || right.empty()) return Err;

    if(left.size() != right.size()) return Err;

    auto calib = calib_.lock();
    if(calib == nullptr) return Err;

    vec<cv::Point3d> unit_lefts;
    vec<cv::Point3d> unit_rights;

    size_t size = left.size();
    for(size_t i = 0 ; i < size ; ++i)
    {
        cv::Point3d unit_left;
        cv::Point3d unit_right;
        
        Geometry::transformCameraPose(left[i],  unit_left,  calib->GetCameraMatrix(), calib->GetdistCoeffs(), true);
        Geometry::transformCameraPose(right[i], unit_right, calib->GetCameraMatrix(), calib->GetdistCoeffs(), true);

        // std::cout << "left Norm: " << norm(unit_left) << std::endl;
        // std::cout << "right Norm: " << norm(unit_right) << std::endl;

        unit_lefts.push_back(unit_left);
        unit_rights.push_back(unit_right);
    }

    for(int i = 0 ; i < unit_lefts.size(); ++i)
    {
        std::cout << "left: " << (unit_lefts[i]) << std::endl;
        std::cout << "right: " << (unit_rights[i]) << std::endl;
    }

    Mat rvec, tvec;
    Geometry::estimateRigidTransformSVD(unit_lefts, unit_rights, rvec, tvec);

    std::cout << rvec << endl;
    std::cout << tvec << endl;

    // double err_sum = 0.0;
    // for (size_t i = 0; i < size; ++i)
    // {
    //     cv::Mat ul = (cv::Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
    //     cv::Mat ur_pred = rvec * ul + tvec;
    //     cv::Point3d ur_actual = unit_rights[i];
    //     cv::Mat ur_act = (cv::Mat_<double>(3,1) << ur_actual.x, ur_actual.y, ur_actual.z);
    //     err_sum += norm(ur_act - ur_pred);
    // }
    // std::cout << "Transform residual: " << err_sum / size << std::endl;

    // std::cout << "Transform Norm: " << norm(tvec) << std::endl;

    double residual_error = 0.0;

    for (size_t i = 0; i < size; ++i)
    {
        cv::Mat v1 = (Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
        cv::Mat v2 = (Mat_<double>(3,1) << unit_rights[i].x, unit_rights[i].y, unit_rights[i].z);

        // R, t 적용
        cv::Mat Rv2 = rvec.t() * v2;

        cv::Mat A(3, 2, CV_64F);
        cv::Mat b(3, 1, CV_64F);

        for (int j = 0; j < 3; ++j)
        {
            A.at<double>(j, 0) = v1.at<double>(j);         // λ1 * v1
            A.at<double>(j, 1) = -Rv2.at<double>(j);       // -λ2 * Rv2
            b.at<double>(j, 0) = tvec.at<double>(j);       // +t
        }

        cv::Mat lambda;
        cv::solve(A, b, lambda, DECOMP_SVD);
        double lambda1 = lambda.at<double>(0);
        double lambda2 = lambda.at<double>(1);

        // if(lambda1 < DEPTH_MIN || lambda1 > DEPTH_MAX) continue;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        cv::Point3d point3D = lambda1 * unit_lefts[i];
        pointcloud.push_back(point3D);

        std::cout << "X : " << point3D.x << " Y : " << point3D.y << " Z : " << point3D.z << std::endl;

        Mat P1 = (Mat_<double>(3,1) << point3D.x, point3D.y, point3D.z);
        Mat P2 = rvec * P1 + tvec;
        Mat diff = P2 - P1;

        residual_error += norm(diff);

        cv::Point3d norm_point3D = point3D / cv::norm(point3D);
        cv::Point3d unit_dir = unit_lefts[i];

        double dot = norm_point3D.dot(unit_dir);  // [-1, 1]
        dot = std::clamp(dot, -1.0, 1.0);         // 안전 처리

        double angle_rad = std::acos(dot);
        double angle_deg = angle_rad * 180.0 / CV_PI;

        std::cout << "Angle error (deg): " << angle_deg << std::endl;

    }

    return residual_error /= size;
}

// double DepthEstimate::MonoDepthEstimate(
//     vec<cv::Point>& left,
//     vec<cv::Point>& right,
//     vec<cv::Point3d>& pointcloud,
//     double baseline_mm
// )
// {
//     const double Err = -1.0;

//     if (left.empty() || right.empty()) return Err;
//     if (left.size() != right.size()) return Err;

//     auto calib = calib_.lock();
//     if (calib == nullptr) return Err;

//     vec<cv::Point3d> unit_lefts;
//     vec<cv::Point3d> unit_rights;

//     const size_t size = left.size();
//     for (size_t i = 0; i < size; ++i)
//     {
//         cv::Point3d ul, ur;

//         // 1. 픽셀 → 정규화된 단위벡터로 변환
//         Geometry::transformCameraPose(left[i],  ul, calib->GetCameraMatrix(), calib->GetdistCoeffs(),true);
//         Geometry::transformCameraPose(right[i], ur, calib->GetCameraMatrix(), calib->GetdistCoeffs(),true);

//         unit_lefts.push_back(ul);
//         unit_rights.push_back(ur);
//     }

//     // 2. 단위벡터 기반 SVD로 SE(3) 추정
//     cv::Mat R, t;
//     Geometry::estimateRigidTransformSVD(unit_lefts, unit_rights, R, t);

//     // 3. 변환 정합 오차 출력
//     double err_sum = 0.0;
//     for (size_t i = 0; i < size; ++i)
//     {
//         cv::Mat ul = (cv::Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
//         cv::Mat ur_actual = (cv::Mat_<double>(3,1) << unit_rights[i].x, unit_rights[i].y, unit_rights[i].z);
//         cv::Mat ur_pred = R * ul + t;
//         err_sum += norm(ur_pred - ur_actual);
//     }
//     std::cout << "[Transform residual]: " << err_sum / size << std::endl;

//     // 4. 스케일 보정
//     // double scale = baseline_mm / cv::norm(t);
//     t = t / cv::norm(t) * baseline_mm;
//     std::cout << "[Scaled t norm]: " << norm(t) << std::endl;

//     // 5. 삼각측량 및 포인트 추정
//     double residual_error = 0.0;
//     pointcloud.clear();

//     for (size_t i = 0; i < size; ++i)
//     {
//         cv::Mat f1 = (cv::Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
//         cv::Mat f2 = (cv::Mat_<double>(3,1) << unit_rights[i].x, unit_rights[i].y, unit_rights[i].z);

//         cv::Mat Rf2 = R * f2;
//         cv::Mat Rt = t;

//         // Ax = b 구성
//         cv::Mat A(3, 2, CV_64F);
//         cv::Mat b(3, 1, CV_64F);
//         for (int j = 0; j < 3; ++j)
//         {
//             A.at<double>(j,0) = f1.at<double>(j);
//             A.at<double>(j,1) = -Rf2.at<double>(j);
//             b.at<double>(j,0) = Rt.at<double>(j);
//         }

//         cv::Mat lambdas;
//         cv::solve(A, b, lambdas, cv::DECOMP_SVD);

//         double lambda1 = lambdas.at<double>(0);  // left 기준
//         cv::Point3d point3D = unit_lefts[i] * lambda1;

//         pointcloud.push_back(point3D);
//         std::cout << "X: " << point3D.x << " Y: " << point3D.y << " Z: " << point3D.z << std::endl;

//         // 6. 재투영 residual 확인
//         cv::Mat P1 = (cv::Mat_<double>(3,1) << point3D.x, point3D.y, point3D.z);
//         cv::Mat P2 = R * P1 + t;
//         residual_error += norm(P2 - P1);  // 트랜스폼된 위치와 원위치 차이
//     }

//     return residual_error / size;
// }

double DepthEstimate::computeBaseLine(cv::Mat& T_base_grip1 , cv::Mat& T_base_grip2)
{
    double baseline = -1.0;

    auto calib = calib_.lock();
    if(calib == nullptr) return baseline;

    Mat G2C; 
    Geometry::homogeneousInverse(calib->Getcam2gripper(), G2C);

    Mat T_base2cam1 = (T_base_grip1) * G2C;
    Mat T_base2cam2 = (T_base_grip2) * G2C;

        // Translation 벡터 추출
    cv::Vec3d p1(T_base2cam1.at<double>(0,3),
                 T_base2cam1.at<double>(1,3),
                 T_base2cam1.at<double>(2,3));

    cv::Vec3d p2(T_base2cam2.at<double>(0,3),
                 T_base2cam2.at<double>(1,3),
                 T_base2cam2.at<double>(2,3));


    cv::Vec3d delta = p2 - p1;
    baseline = cv::norm(delta);

    return baseline;
}