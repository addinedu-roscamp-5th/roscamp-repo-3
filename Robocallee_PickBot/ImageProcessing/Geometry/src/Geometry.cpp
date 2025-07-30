#include "Geometry.hpp"

using namespace geometry;
using namespace cv;




static bool Geometry::transformCameraPose(
    cv::Point point2d, // mm
    cv::Point3d point3d, // mm
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const bool useUnDist = false
)
{
    
}


static bool Geometry::estimateRigidTransformSVD(
    const std::vector<cv::Point3d>& P1,
    const std::vector<cv::Point3d>& P2,
    cv::Mat& R, cv::Mat& t)
{
    if (P1.size() != P2.size() || P1.size() < 3) return false;

    // 1. 평균 중심 정렬
    Point3d c1(0, 0, 0), c2(0, 0, 0);
    for (int i = 0; i < P1.size(); ++i) {
        c1 += P1[i];
        c2 += P2[i];
    }
    c1 *= (1.0 / P1.size());
    c2 *= (1.0 / P2.size());

    vector<Point3d> q1, q2;
    for (int i = 0; i < P1.size(); ++i) {
        q1.push_back(P1[i] - c1);
        q2.push_back(P2[i] - c2);
    }

    // 2. 공분산 행렬 계산
    Mat H = Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < P1.size(); ++i) {
        Mat q1_mat = (Mat_<double>(3,1) << q1[i].x, q1[i].y, q1[i].z);
        Mat q2_mat = (Mat_<double>(1,3) << q2[i].x, q2[i].y, q2[i].z);
        H += q1_mat * q2_mat; // 3x3
    }

    // 3. SVD 분해: H = U * S * Vt
    Mat U, S, Vt;
    SVD::compute(H, S, U, Vt);

    R = Vt.t() * U.t();

    // 반사 방지 (det(R) < 0)
    if (determinant(R) < 0) {
        Vt.row(2) *= -1;
        R = Vt.t() * U.t();
    }

    // 4. 이동 벡터 t = c2 - R * c1
    Mat c1_mat = (Mat_<double>(3,1) << c1.x, c1.y, c1.z);
    Mat c2_mat = (Mat_<double>(3,1) << c2.x, c2.y, c2.z);
    t = c2_mat - R * c1_mat;

    return true;
}