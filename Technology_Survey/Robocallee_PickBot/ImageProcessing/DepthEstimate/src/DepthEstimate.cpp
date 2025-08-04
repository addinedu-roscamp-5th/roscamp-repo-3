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

double DepthEstimate::MonoDepthEstimate(
            cv::Mat& left,
            cv::Mat& right,
            cv::Mat& DepthMap,
            double baseline)
{
    const double err = -1;
    if(left.empty() || right.empty()) return err;

    auto detector = detector_.lock();
    if(detector == nullptr) return err;

    auto calib = calib_.lock();
    if(calib == nullptr) return err;

    //1. ORB 특징점 추출 및 매칭
    Ptr<ORB> orb = ORB::create(2000);
    vector<KeyPoint> kp1, kp2;
    Mat desc1, desc2;
    orb->detectAndCompute(left, noArray(), kp1, desc1);
    orb->detectAndCompute(right, noArray(), kp2, desc2);

    BFMatcher matcher(NORM_HAMMING);
    vector<DMatch> matches;
    matcher.match(desc1, desc2, matches);

    // 2. 좋은 매칭만 필터링
    double max_dist = 0.0;
    for (auto& m : matches)
    {
        max_dist = max(max_dist, (double)m.distance);
    }

    vector<Point3d> ptsls, ptsrs;
    for (auto& m : matches) 
    {
        if (m.distance < 0.6 * max_dist) 
        {
            Point3d ptsl , ptsr;

            Geometry::transformCameraPose(kp1[m.queryIdx].pt,ptsl,calib->GetCameraMatrix(),calib->GetdistCoeffs());
            Geometry::transformCameraPose(kp2[m.trainIdx].pt,ptsr,calib->GetCameraMatrix(),calib->GetdistCoeffs());

            ptsls.push_back(ptsl);
            ptsrs.push_back(ptsr);
        }
    }

    if (ptsls.size() < 8)
    {
        cerr << "매칭된 점이 너무 적습니다." << endl;
        return err;
    }

    // 3. 카메라 간의 변환 행렬 계산
    Mat T_se3, rvec, tvec;
    Geometry::estimateRigidTransformSVD(ptsls, ptsrs, rvec, tvec);
    Geometry::composeTransform(rvec, tvec, T_se3);

    DepthMap = Mat::zeros(left.size(), CV_64F);

    double residual_error = 0.0;

    for (size_t i = 0; i < ptsls.size(); ++i)
    {
        cv::Mat v1 = (Mat_<double>(3,1) << ptsls[i].x, ptsls[i].y, ptsls[i].z);
        cv::Mat v2 = (Mat_<double>(3,1) << ptsrs[i].x, ptsrs[i].y, ptsrs[i].z);

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

        double depth = lambda.at<double>(0);

        Mat temp = depth * v1;
        double depth_real = temp.at<double>(2);

        
        if (depth > 0 && depth < 10000) // 합리적인 범위 필터링
        {
            int x = (int)kp1[matches[i].queryIdx].pt.x;
            int y = (int)kp1[matches[i].queryIdx].pt.y;

            cout << "X : " << x <<" y : " << " z : " << depth_real << endl;

            if (x >= 0 && x < DepthMap.cols && y >= 0 && y < DepthMap.rows)
                DepthMap.at<double>(y, x) = depth;
        }

        Mat P1 = lambda.at<double>(0) * (Mat_<double>(3,1) << ptsls[i].x, ptsls[i].y, ptsls[i].z);
        Mat P2 = rvec * P1 + tvec;
        Mat diff = P2 - (Mat_<double>(3,1) << ptsrs[i].x, ptsrs[i].y, ptsrs[i].z);
        residual_error += norm(diff);

    }
    cv::Mat mat_8u;
    cv::normalize(DepthMap, mat_8u, 0, 255, cv::NORM_MINMAX);
    mat_8u.convertTo(mat_8u, CV_8U);

    cv::imwrite("debug_img.bmp", mat_8u);
    residual_error /= ptsls.size();

    return residual_error;
}

double DepthEstimate::computeBaseLine(cv::Mat& T_base_grip1 , cv::Mat& T_base_grip2)
{
    double baseline = -1.0;

    auto calib = calib_.lock();
    if(calib == nullptr) return baseline;

    Mat G2C; 
    Geometry::homogeneousInverse(calib->Getcam2gripper(), G2C);

    Mat T_cam1 = (T_base_grip1.inv()) * G2C;
    Mat T_cam2 = (T_base_grip2.inv()) * G2C;

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