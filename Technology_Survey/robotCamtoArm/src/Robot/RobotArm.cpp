#include "../../include/Robot/RobotArm.hpp"

using namespace ARM;

RobotArm::RobotArm()
{

}

RobotArm::~RobotArm()
{
    
}

bool RobotArm::dh_to_transform(const double a, const  double alpha, const double d, const double theta , cv::Mat& T)
{
    T = (cv::Mat_<double>(4,4) <<
        cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
        sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0,           sin(alpha),               cos(alpha),              d,
        0,           0,                        0,                       1
    );

    return true;
}


 bool RobotArm::compute_forward_kinematics(
            const std::vector<double>& joint_angles,
            const std::vector<std::array<double, DH_PARAM>> DH,
            cv::Mat& FK
        )
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

bool RobotArm::decomposeTransform(const cv::Mat& T_se3 , cv::Mat& rvec , cv::Mat& tvec)
{
    if(T_se3.empty()|| T_se3.rows != 4 || T_se3.cols != 4) return false;

    cv::Mat R = T_se3(cv::Range(0, 3), cv::Range(0, 3)).clone();

    tvec = T_se3(cv::Range(0, 3), cv::Range(3, 4)).clone();

    cv::Rodrigues(R, rvec);

    return true;

}

bool RobotArm::composeTransform(const cv::Mat& rvec, const cv::Mat& tvec, cv::Mat& T_se3)
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



bool RobotArm::degreesToRadians(const std::vector<double>& degrees ,std::vector<double> &radians)
{
    radians.reserve(degrees.size());  // 성능 최적화

    for (double deg : degrees)
        radians.push_back(deg * DEG2RAD);
    
    return true;
}

bool RobotArm::toHomogeneous(const cv::Mat& vecNx1, cv::Mat& vecNpx1)
{
    if(vecNx1.empty()) return false;
    
    int N = vecNx1.rows;
    int type = vecNx1.type();

    cv::Mat vecNp1x1(N + 1, 1, type);

    // 데이터 복사
    for (int i = 0; i < N; ++i)
    {
        if (type == CV_64F)
            vecNp1x1.at<double>(i, 0) = vecNx1.at<double>(i, 0);
        else if (type == CV_32F)
            vecNp1x1.at<float>(i, 0) = vecNx1.at<float>(i, 0);
        // else
        //     CV_Error(cv::Error::StsUnsupportedFormat, "Only float and double are supported");
    }

    // 마지막 항 추가
    if (type == CV_64F)
        vecNp1x1.at<double>(N, 0) = 1.0;
    else if (type == CV_32F)
        vecNp1x1.at<float>(N, 0) = 1.0f;

    vecNpx1 = vecNp1x1.clone();
    return true;
}

bool RobotArm::getTransformMatrixFromCoords(const std::vector<float>& coords , cv::Mat& T)
{
    if(coords.size() != 6) return false;

    // 1. 위치 (단위 그대로 사용, mm 기준)
    double x = coords[0];
    double y = coords[1];
    double z = coords[2];

    // 2. 오일러 각도 (deg → rad)
    double rx = coords[3] * DEG2RAD; // Roll (X)
    double ry = coords[4] * DEG2RAD; // Pitch (Y)
    double rz = coords[5] * DEG2RAD; // Yaw (Z)

    // 3. 각각의 축 회전 행렬 (XYZ 순서)
    cv::Mat Rx = (cv::Mat_<double>(3,3) <<
        1, 0,         0,
        0, cos(rx), -sin(rx),
        0, sin(rx),  cos(rx));

    cv::Mat Ry = (cv::Mat_<double>(3,3) <<
        cos(ry), 0, sin(ry),
        0,       1, 0,
       -sin(ry), 0, cos(ry));

    cv::Mat Rz = (cv::Mat_<double>(3,3) <<
        cos(rz), -sin(rz), 0,
        sin(rz),  cos(rz), 0,
        0,        0,       1);

    // 4. 회전 행렬 조합: R = Rz * Ry * Rx  (XYZ 순서)
    cv::Mat R = Rz * Ry * Rx;

    // 5. 4x4 변환 행렬 구성
    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3))); // 좌측 상단 3x3
    H.at<double>(0, 3) = x;
    H.at<double>(1, 3) = y;
    H.at<double>(2, 3) = z;

    T = H.clone();

    return false;
}

bool RobotArm::homogeneousInverse(const cv::Mat& T, cv::Mat& invT)
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