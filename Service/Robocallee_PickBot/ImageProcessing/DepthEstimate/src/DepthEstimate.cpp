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
        
        Geometry::transformCameraPose(left[i],  unit_left,  calib->GetCameraMatrix(), calib->GetdistCoeffs());
        Geometry::transformCameraPose(right[i], unit_right, calib->GetCameraMatrix(), calib->GetdistCoeffs());

        std::cout << "left Norm: " << norm(unit_left) << std::endl;
        std::cout << "right Norm: " << norm(unit_right) << std::endl;

        unit_lefts.push_back(unit_left);
        unit_rights.push_back(unit_right);
    }

    Mat T_se3, invT_se3, rvec, tvec, invrvec, invtvec;
    Geometry::estimateRigidTransformSVD(unit_lefts, unit_rights, rvec, tvec);

    double err_sum = 0.0;
    for (size_t i = 0; i < size; ++i)
    {
        cv::Mat ul = (cv::Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
        cv::Mat ur_pred = rvec * ul + tvec;
        cv::Point3d ur_actual = unit_rights[i];
        cv::Mat ur_act = (cv::Mat_<double>(3,1) << ur_actual.x, ur_actual.y, ur_actual.z);
        err_sum += norm(ur_act - ur_pred);
    }
    std::cout << "Transform residual: " << err_sum / size << std::endl;

    std::cout << "Transform Norm: " << norm(tvec) << std::endl;

    cv::Mat t_dir = tvec / cv::norm(tvec);    
    double scale = baseline;

    double residual_error = 0.0;

    for (size_t i = 0; i < size; ++i)
    {
        cv::Mat v1 = (Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
        cv::Mat v2 = (Mat_<double>(3,1) << unit_rights[i].x, unit_rights[i].y, unit_rights[i].z);

        // R, t 적용
        Mat v2p = rvec.t() * v2;
        Mat tp = rvec.t() * tvec;

        Mat A(3, 2, CV_64F), b(3, 1, CV_64F);
        for (int j = 0; j < 3; ++j)
        {
            A.at<double>(j,0) = v1.at<double>(j);
            A.at<double>(j,1) = -v2p.at<double>(j);
            b.at<double>(j,0) = -tp.at<double>(j);
        }

        Mat lambda;
        solve(A, b, lambda, DECOMP_SVD);
        double lambda_l = lambda.at<double>(0);

        // if(lambda_l < DEPTH_MIN || lambda_l > DEPTH_MAX) continue;

        cv::Point3d point3D = lambda_l * scale * unit_lefts[i];
        pointcloud.push_back(point3D);

        std::cout << "X : " << point3D.x << " Y : " << point3D.y << " Z : " << point3D.z << std::endl;

        Mat P1 = (Mat_<double>(3,1) << point3D.x, point3D.y, point3D.z);
        Mat P2 = rvec * P1 + tvec;
        Mat diff = P2 - P1;

        residual_error += norm(diff);
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

//     // 1. 입력 검증
//     if(left.empty() || right.empty()) return Err;
//     if(left.size() != right.size()) return Err;

//     auto calib = calib_.lock();
//     if(calib == nullptr) return Err;

//     // 2. 픽셀 → 카메라 좌표계 → 단위 벡터
//     vec<cv::Point3d> unit_lefts, unit_rights;
//     for(size_t i = 0; i < left.size(); ++i)
//     {
//         cv::Point3d uL, uR;
//         Geometry::transformCameraPose(left[i],  uL, calib->GetCameraMatrix(), calib->GetdistCoeffs());
//         Geometry::transformCameraPose(right[i], uR, calib->GetCameraMatrix(), calib->GetdistCoeffs());

//         // 정규화하여 단위벡터로
//         unit_lefts.push_back(uL / cv::norm(uL));
//         unit_rights.push_back(uR / cv::norm(uR));
//     }

//     // 3. SE(3) 추정: R, t 구하기 (단위벡터 기반)
//     cv::Mat rvec, tvec;
//     Geometry::estimateRigidTransformSVD(unit_lefts, unit_rights, rvec, tvec);

//     std::cout << "[Info] 추정된 이동 벡터 norm (단위 없음): " << cv::norm(tvec) << std::endl;

//     // 4. 스케일 보정: 방향만 유지하고 baseline으로 스케일 고정
//     cv::Mat t_dir = tvec / cv::norm(tvec);
//     tvec = t_dir * baseline_mm;  // 단위: mm

//     double residual_error = 0.0;

//     // 5. 삼각측량: λ 계산 후 λ * 단위벡터 = 3D 포인트
//     for(size_t i = 0; i < unit_lefts.size(); ++i)
//     {
//         cv::Mat v1 = (cv::Mat_<double>(3,1) << unit_lefts[i].x, unit_lefts[i].y, unit_lefts[i].z);
//         cv::Mat v2 = (cv::Mat_<double>(3,1) << unit_rights[i].x, unit_rights[i].y, unit_rights[i].z);

//         // 우측 기준으로 좌측 벡터 정렬
//         cv::Mat v2p = rvec.t() * v2;
//         cv::Mat tp = rvec.t() * tvec;

//         // 선형 방정식 구성: Aλ = b
//         cv::Mat A(3, 2, CV_64F), b(3, 1, CV_64F);
//         for(int j = 0; j < 3; ++j)
//         {
//             A.at<double>(j,0) = v1.at<double>(j);
//             A.at<double>(j,1) = -v2p.at<double>(j);
//             b.at<double>(j,0) = -tp.at<double>(j);
//         }

//         cv::Mat lambda;
//         solve(A, b, lambda, cv::DECOMP_SVD);

//         double lambda1 = lambda.at<double>(0);  // 좌측 단위벡터의 람다

//         // mm 단위 3D 포인트 계산
//         cv::Point3d point3D = lambda1 * unit_lefts[i];
//         pointcloud.push_back(point3D);

//         // 재투영 오차 계산 (R, t 기반 양쪽 포인트 비교)
//         cv::Mat P1 = (cv::Mat_<double>(3,1) << point3D.x, point3D.y, point3D.z);
//         cv::Mat P2 = rvec * P1 + tvec;
//         cv::Mat diff = P2 - P1;
//         residual_error += norm(diff);

//         std::cout << "[3D Point] X: " << point3D.x << " Y: " << point3D.y << " Z: " << point3D.z << std::endl;
//     }

//     return residual_error / static_cast<double>(unit_lefts.size());
// }

double DepthEstimate::MonoDepthEstimate(
    vec<cv::Point>& left,
    vec<cv::Point>& right,
    vec<cv::Point3d>& pointcloud,
    double baseline_mm
)
{
    const double Err = -1.0;

    if(left.empty() || right.empty()) return Err;
    if(left.size() != right.size()) return Err;

    auto calib = calib_.lock();
    if(calib == nullptr) return Err;

    // 1. 내부 파라미터
    cv::Mat K = calib->GetCameraMatrix();     // 3x3
    cv::Mat D = calib->GetdistCoeffs();       // 왜곡 계수 (사용 안함 가정)
    
    // 2. 좌/우 카메라 포즈 정의 (기준: 좌측 카메라 원점 기준)
    // P1 = K * [I | 0], P2 = K * [R | t]
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = (cv::Mat_<double>(3,1) << -baseline_mm, 0, 0);  // X축 기준 baseline (mm)

    cv::Mat Rt1, Rt2;
    cv::hconcat(R, cv::Mat::zeros(3,1,CV_64F), Rt1);  // [I | 0]
    cv::hconcat(R, t, Rt2);                           // [I | t]

    cv::Mat P1 = K * Rt1;
    cv::Mat P2 = K * Rt2;

    // 3. 좌/우 픽셀 좌표 → Point2f로 변환
    std::vector<cv::Point2f> pts1, pts2;
    for (size_t i = 0; i < left.size(); ++i)
    {
        pts1.push_back(cv::Point2f(left[i]));
        pts2.push_back(cv::Point2f(right[i]));
    }

    // 4. 삼각측량
    cv::Mat points4D;  // 4xN (동차좌표)
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

    // 5. 동차좌표 → 3D (정규화) → mm 단위 보존
    pointcloud.clear();
    double total_z = 0.0;

    for (int i = 0; i < points4D.cols; ++i)
    {
        cv::Mat col = points4D.col(i);
        cv::Point3d pt;

        pt.x = col.at<float>(0) / col.at<float>(3);
        pt.y = col.at<float>(1) / col.at<float>(3);
        pt.z = col.at<float>(2) / col.at<float>(3);  // 단위: mm (baseline을 mm로 설정했기 때문)

        pointcloud.push_back(pt);
        total_z += pt.z;

        std::cout << "[3D Point] X: " << pt.x << " Y: " << pt.y << " Z: " << pt.z << std::endl;
    }

    return total_z / static_cast<double>(points4D.cols);  // 평균 Z값 반환 (평균 깊이)
}

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