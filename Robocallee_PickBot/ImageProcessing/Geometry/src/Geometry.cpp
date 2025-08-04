#include "Geometry.hpp"

using namespace geometry;
using namespace Commondefine;
using namespace cv;
using namespace std;


double Geometry::transformCameraPose(
    cv::Point point2d, // mm
    cv::Point3d& point3d, // mm
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const bool useUnDist
)
{
    if(cameraMatrix.empty() || distCoeffs.empty()) return -1;
	
    cv::Point3d unitVec;

	if(useUnDist)
	{
		cv::Mat undistorted2f;

		// 왜곡 보정 → 내부 파라미터까지 적용됨 → z=1 평면상의 좌표
		cv::undistortPoints(pointToMat(point2d), undistorted2f, cameraMatrix, distCoeffs);
        
        cv::Point2f pt = matToPoint<Point2f>(undistorted2f);

        // 동차 좌표계로 확장
        cv::Point3d vec3D(pt.x, pt.y, 1.0);

        // 단위 벡터로 정규화
        double norm = cv::norm(vec3D);
        unitVec = vec3D * (1.0 / norm);
	}
	else
	{
		cv::Mat invk = cameraMatrix.inv();

        cv::Mat uv = (cv::Mat_<double>(3,1) << point2d.x, point2d.y, 1.0);
        cv::Mat dir = cameraMatrix.inv() * uv;
        
        double x = dir.at<double>(0, 0) / dir.at<double>(2, 0);
        double y = dir.at<double>(1, 0) / dir.at<double>(2, 0);

        cv::Point3d vec3D(x, y, 1.0);

        // 단위 벡터로 정규화
        double norm = cv::norm(vec3D);
        unitVec = vec3D * (1.0 / norm);
    }

    point3d = unitVec;

    return -1;
};

bool Geometry::estimateRigidTransformSVD(
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

bool Geometry::composeTransform(const cv::Mat& rvec, const cv::Mat& tvec, cv::Mat& T_se3)
{
    // 입력 유효성 확인
    if ((rvec.rows != 3 || (rvec.cols != 1 && rvec.cols != 3)) || rvec.type() != CV_64F ||
        tvec.rows != 3 || tvec.cols != 1 || tvec.type() != CV_64F)
    {
        std::cerr << "[Error] rvec은 3x1 또는 3x3, tvec은 3x1 CV_64F 형식이어야 합니다.\n";
        return false;
    }

    cv::Mat R;

    if (rvec.cols == 1)
    {
        cv::Rodrigues(rvec, R);
    }
    else if (rvec.cols == 3)
    {
        R = rvec.clone();  // 복사해서 사용
    } else
    {
        std::cerr << "[Error] rvec는 3x1 또는 3x3 행렬이어야 합니다.\n";
        return false;
    }

    // 2. 4x4 SE(3) 행렬 생성
    T_se3 = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T_se3(cv::Range(0, 3), cv::Range(0, 3)));       // 회전행렬 R
    tvec.copyTo(T_se3(cv::Range(0, 3), cv::Range(3, 4)));    // 이동벡터 t

    return true;
}

bool Geometry::decomposeTransform(const cv::Mat& T_se3 , cv::Mat& rvec , cv::Mat& tvec , bool rodrigues)
{
    if(T_se3.empty()|| T_se3.rows != 4 || T_se3.cols != 4) return false;

    cv::Mat R = T_se3(cv::Range(0, 3), cv::Range(0, 3)).clone();

    tvec = T_se3(cv::Range(0, 3), cv::Range(3, 4)).clone();

    if(rodrigues)
        cv::Rodrigues(R, rvec);
    else
        rvec = R.clone();

    return true;
}


bool Geometry::dh_to_transform(const double a, const  double alpha, const double d, const double theta , cv::Mat& T)
{
    T = (cv::Mat_<double>(4,4) <<
            cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
            sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
            0,           sin(alpha),               cos(alpha),              d,
            0,           0,                        0,                       1
        );

    return true;
}

bool Geometry::compute_forward_kinematics(const std::vector<double>& joint_angles, const std::vector<std::array<double, 4>> DH, cv::Mat& FK)
{
    if (joint_angles.size() != DH.size())
    {
        std::cerr << "[FK ERROR] 조인트 개수와 DH 테이블 크기가 일치하지 않습니다." << std::endl;
        return false;
    }

    FK = cv::Mat::eye(4, 4, CV_64F);  // 단위 행렬 초기화

    double a , alpha , d , offset, theta;

    for (size_t i = 0; i < DH.size(); ++i)
    {
        a        = DH[i][Params::a];
        alpha    = DH[i][Params::alpha];
        d        = DH[i][Params::d];
        offset   = DH[i][Params::theta_offset];
        theta    = joint_angles[i] + offset;

        cv::Mat T_i;
        
        dh_to_transform(a, alpha, d, theta, T_i);
        
        FK = FK * T_i;
    }

    return true;
}

bool Geometry::homogeneousInverse(const cv::Mat& T, cv::Mat& invT)
{
    if(T.rows != 4 && T.cols != 4) return false;

    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));
    cv::Mat Rt = R.t();
    cv::Mat tinv = -Rt * t;
    cv::Mat Tinv = cv::Mat::eye(4, 4, T.type());

    Rt.copyTo(Tinv(cv::Rect(0, 0, 3, 3)));
    tinv.copyTo(Tinv(cv::Rect(3, 0, 1, 3)));

    invT = Tinv.clone();
    
    return true;
}