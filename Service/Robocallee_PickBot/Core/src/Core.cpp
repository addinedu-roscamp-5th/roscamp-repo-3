#include "Core.hpp"

using namespace core;
using namespace Integrated;
using namespace Commondefine;
using namespace geometry;
using namespace Calib;
using namespace cap;
using namespace RA;
using namespace OB;
using namespace cv;

Core::Core(Logger::s_ptr log , Integrated::w_ptr<interface::RosInterface> Interface)
    :log_(log) , Interface_(Interface)
{
    log_->Log(INFO,"Core 생성...");
}

Core::~Core()
{

}

bool Core::initialize()
{
    log_->Log(INFO,"Core Initialize...");

    bool flag = false;

    RobotArm_ = make_uptr<RobotArm>();
    flag = RobotArm_->Connet(JETCOBOT,BAUDRATE);
    if(!flag)
    {
        log_->Log(FATAL,"Robot Arm 초기화 Fail");
        return false;
    } 

    // capture_ = make_uptr<ImageCapture>(log_);
    // flag = capture_->open(_CAM_INDEX_, CAP_V4L2);
    // if(!flag)
    // {
    //     log_->Log(FATAL,"ImageCapture Camera open Fail");
    //     return false;
    // }

    calib_ = make_sptr<Calibrator>(_CALIB_FILE_DIR_);

    detector_ = make_sptr<ArUcoMakerDetector>();
    detector_->initDetector();

    depth_ = make_uptr<depth::DepthEstimate>(detector_, calib_);

    log_->Log(INFO,"Core Initialize Done");
    return true;
}

bool Core::computeDepth()
{
    std::vector<double> left_joint = {0.028, -0.103, -0.494, -0.615, -0.018, 0.79};
    std::vector<double> right_joint = {-0.095, -0.241, -0.348, -0.679, 0.018, 0.664};

    cv::Mat lT , rT;
    Geometry::compute_forward_kinematics(left_joint, mycobot280_dh_params, lT);
    Geometry::compute_forward_kinematics(right_joint, mycobot280_dh_params, rT);

    double baseline = depth_->computeBaseLine(lT, rT);

    vec<OB::object2d> leftob,rightob;
    cv::Mat left = cv::imread("../../../DepthTEST/img_pos1.bmp");
    cv::Mat right = cv::imread("../../../DepthTEST/img_pos2.bmp");

    Mat k = calib_->GetCameraMatrix();
    Mat d = calib_->GetdistCoeffs();

    vec<Mat> rvec , tvec;
    detector_->EstimatePose(left, k, d, 50, rvec, tvec);

    std::cout << tvec[0] << std::endl;

    detector_->FindObject(left,leftob);
    detector_->FindObject(right,rightob);

    vec<cv::Point3d> pointcloud;
    double error = depth_->MonoDepthEstimate(leftob[0].points, rightob[0].points, pointcloud, baseline);

    std::cout << error << std::endl;

    // cv::Mat T = calib_->Getcam2gripper();

    // int N = pointcloud.size();
    // cv::Mat points4xN(4, N, CV_64F);  // 4행 N열

    // for (int i = 0; i < N; ++i)
    // {
    //     points4xN.at<double>(0, i) = pointcloud[i].x;
    //     points4xN.at<double>(1, i) = pointcloud[i].y;
    //     points4xN.at<double>(2, i) = pointcloud[i].z;
    //     points4xN.at<double>(3, i) = 1.0;
    // }

    //  cv::Mat transformed4xN = T * points4xN;

    // // 5. 다시 std::vector<cv::Point3d>로 복원
    // std::vector<cv::Point3d> transformed_points;
    // for (int i = 0; i < N; ++i)
    // {
    //     double x = transformed4xN.at<double>(0, i);
    //     double y = transformed4xN.at<double>(1, i);
    //     double z = transformed4xN.at<double>(2, i);
    //     double w = transformed4xN.at<double>(3, i);  // 일반적으로 1

    //     // w ≠ 1일 경우 정규화
    //     if (std::abs(w) > 1e-6)
    //     {
    //         x /= w;
    //         y /= w;
    //         z /= w;
    //     }

    //     transformed_points.emplace_back(x, y, z);
    // }

    // // 6. 출력 확인
    // for (const auto& pt : transformed_points)
    // {
    //     std::cout << "Transformed Point: "
    //               << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
    // }

    return true;

}

