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

// double DepthEstimate::MonoDepthEstimate(
//             cv::Mat& left,
//             cv::Mat& right,
//             cv::Mat& DepthMap,
//             double baseline)
// {
//     const double err = -1;
//     if(left.empty() || right.empty()) return err;

//     auto detector = detector_.lock();
//     if(detector == nullptr) return err;

//     auto calib = calib_.lock();
//     if(calib == nullptr) return err;

//     Mat g_left , g_right;
//     cvtColor(left, g_left, COLOR_BGR2GRAY);
//     cvtColor(right, g_right, COLOR_BGR2GRAY);

//     threshold(g_left, g_left, 50, 255, THRESH_BINARY_INV);
//     threshold(g_right, g_right, 50, 255, THRESH_BINARY_INV);
//     cv::imwrite("g_left.png", g_left);
//     cv::imwrite("g_right.png", g_right);


//     //1. ORB 특징점 추출 및 매칭
//     Ptr<ORB> orb = ORB::create(2000);
//     vector<KeyPoint> kp1, kp2;
//     Mat desc1, desc2;
//     orb->detectAndCompute(g_left, noArray(), kp1, desc1);
//     orb->detectAndCompute(g_right, noArray(), kp2, desc2);

//     BFMatcher matcher(NORM_HAMMING);
//     vector<DMatch> matches;
//     matcher.match(desc1, desc2, matches);

//     std::sort(matches.begin(), matches.end(), [](const DMatch& a, const DMatch& b)
//     {
//         return a.distance < b.distance;
//     });

//     // 4. 상위 10개만 추출
//     int top_k = 10;
//     vector<DMatch> top_matches(matches.begin(), matches.begin() + min(top_k, (int)matches.size()));

//     Mat match_img;
//     drawMatches(g_left, kp1, g_right, kp2, top_matches, match_img);
//     cv::imwrite("orb_matches_raw.png", match_img);

//     vector<Point3d> ptsls, ptsrs;
//     for (auto& m : top_matches) 
//     {
//         //if (m.distance < 0.2 * max_dist) 
//         {
//             Point3d ptsl , ptsr;

//             Geometry::transformCameraPose(kp1[m.queryIdx].pt, ptsl, calib->GetCameraMatrix(), calib->GetdistCoeffs());
//             Geometry::transformCameraPose(kp2[m.trainIdx].pt, ptsr, calib->GetCameraMatrix(), calib->GetdistCoeffs());

//             ptsls.push_back(ptsl);
//             ptsrs.push_back(ptsr);
//         }
//     }

//     if (ptsls.size() < 8)
//     {
//         cerr << "매칭된 점이 너무 적습니다." << endl;
//         return err;
//     }

//     // 3. 단위 벡터를 사용해 카메라 간의 변환 행렬 계산 l -> r
//     Mat T_se3, invT_se3, rvec, tvec, invrvec, invtvec;
//     Geometry::estimateRigidTransformSVD(ptsls, ptsrs, rvec, tvec);

//     //변환 행렬을 mm 단위로 변경
//     tvec *= (baseline / norm(tvec));

//     Geometry::composeTransform(rvec, tvec, T_se3);
//     Geometry::homogeneousInverse(T_se3, invT_se3);
//     Geometry::decomposeTransform(invT_se3, invrvec, invtvec);

//     double residual_error = 0.0;
//     for (size_t i = 0; i < ptsls.size(); ++i)
//     {
//         cv::Mat v1 = (Mat_<double>(3,1) << ptsls[i].x, ptsls[i].y, ptsls[i].z);
//         cv::Mat v2 = (Mat_<double>(3,1) << ptsrs[i].x, ptsrs[i].y, ptsrs[i].z);

//         // R, t 적용
//         Mat v2p = invrvec * v2;
//         Mat tp = invrvec * invtvec;

//         Mat A(3, 2, CV_64F), b(3, 1, CV_64F);
//         for (int j = 0; j < 3; ++j)
//         {
//             A.at<double>(j,0) = v1.at<double>(j);
//             A.at<double>(j,1) = -v2p.at<double>(j);
//             b.at<double>(j,0) = -tp.at<double>(j);
//         }

//         Mat lambda;
//         solve(A, b, lambda, DECOMP_SVD);

//         double depth = lambda.at<double>(0);

//         if (depth > 0 && depth < 10000) // 합리적인 범위 필터링
//         {
//             int x = (int)kp1[matches[i].queryIdx].pt.x;
//             int y = (int)kp1[matches[i].queryIdx].pt.y;

//             cout << "X : " << x <<" Y : " << y << " Z : " << depth << endl;

//             if (x >= 0 && x < DepthMap.cols && y >= 0 && y < DepthMap.rows)
//                 DepthMap.at<double>(y, x) = depth;
//         }

//         Mat P1 = lambda.at<double>(0) * (Mat_<double>(3,1) << ptsls[i].x, ptsls[i].y, ptsls[i].z);
//         Mat P2 = rvec * P1 + tvec;
//         Mat diff = P2 - (Mat_<double>(3,1) << ptsrs[i].x, ptsrs[i].y, ptsrs[i].z);
//         residual_error += norm(diff);
//     }

//     residual_error /= ptsls.size();

//     return residual_error;
// }

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
        double depth = lambda.at<double>(0) * baseline;

        if(depth < DEPTH_MIN || depth > DEPTH_MAX) continue;

        cv::Point3d point3D = depth * unit_lefts[i];
        pointcloud.push_back(point3D);

        Mat P1 = (Mat_<double>(3,1) << point3D.x, point3D.y, point3D.z);
        Mat P2 = rvec * P1 + tvec;
        Mat diff = P2 - P1;

        residual_error += norm(diff);
    }

    return residual_error /= size;
}

double DepthEstimate::computeBaseLine(cv::Mat& T_base_grip1 , cv::Mat& T_base_grip2)
{
    double baseline = -1.0;

    auto calib = calib_.lock();
    if(calib == nullptr) return baseline;

    Mat G2C; 
    Geometry::homogeneousInverse(calib->Getcam2gripper(), G2C);

    Mat T_cam1 = (T_base_grip1) * G2C;
    Mat T_cam2 = (T_base_grip2) * G2C;

        // Translation 벡터 추출
    cv::Vec3d p1(T_cam1.at<double>(0,3),
                 T_cam1.at<double>(1,3),
                 T_cam1.at<double>(2,3));

    cv::Vec3d p2(T_cam2.at<double>(0,3),
                 T_cam2.at<double>(1,3),
                 T_cam2.at<double>(2,3));


    cv::Vec3d delta = p2 - p1;
    baseline = cv::norm(delta);

    return baseline;
}