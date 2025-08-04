#include "Core.hpp"

using namespace core;
using namespace Integrated;
using namespace Commondefine;
using namespace geometry;
using namespace Calib;
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

    video_ = make_uptr<VideoCapture>();
    flag = video_->open(_CAM_INDEX_, CAP_V4L2);
    if(!flag)
    {
        log_->Log(FATAL,"VideoCapture 초기화 Fail");
        return false;
    } 

    calib_ = make_sptr<Calibrator>(_CALIB_FILE_DIR_);

    detector_ = make_sptr<ArUcoMakerDetector>();
    detector_->initDetector();

    depth_ = make_uptr<depth::DepthEstimate>(detector_, calib_);

    log_->Log(INFO,"Core Initialize Done");
    return true;
}

bool Core::computeDepth()
{
    std::vector<double> left_joint = {0.021, 0.015, -1.224, -0.29, 0.051, 0.798};
    std::vector<double> right_joint = {-0.086, -0.073, -1.163, -0.32, 0.051, 0.69};

    cv::Mat lT , rT;
    Geometry::compute_forward_kinematics(left_joint, mycobot280_dh_params, lT);
    Geometry::compute_forward_kinematics(right_joint, mycobot280_dh_params, rT);

    double baseline = depth_->computeBaseLine(lT, rT);

    cv::Mat left = cv::imread("calib_file/img_pos1.bmp");
    cv::Mat right = cv::imread("calib_file/img_pos2.bmp");
    cv::Mat depthmap;
    depth_->MonoDepthEstimate(left,right,depthmap,baseline);

    return true;

}

