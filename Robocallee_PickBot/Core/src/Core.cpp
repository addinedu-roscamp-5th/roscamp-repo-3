#include "Core.hpp"

using namespace core;
using namespace Integrated;
using namespace Commondefine;
using namespace Calib;
using namespace RA;
using namespace cv;

Core::Core(Logger::s_ptr log , Integrated::w_ptr<interface::RosInterface> Interface)
    :log_(log) , Interface_(Interface)
{
    log_->Log(INFO,"Core Initialize...");
    bool flag = false;

    RobotArm_ = make_uptr<RobotArm>();
    flag = RobotArm_->Connet(JETCOBOT,BAUDRATE);
    if(!flag) log_->Log(FATAL,"Robot Arm 초기화 Fail");

    video_ = make_uptr<VideoCapture>();
    flag = video_->open(_CAM_INDEX_, CAP_V4L2);
    if(!flag) log_->Log(FATAL,"VideoCapture 초기화 Fail");

    calib_ = make_uptr<Calibrator>(_CALIB_FILE_DIR_);

    depth::DepthEstimate* depth = new depth::DepthEstimate();

    Mat imgL = imread("calib_file/img_pos1.bmp", IMREAD_GRAYSCALE);
    Mat imgR = imread("calib_file/img_pos2.bmp", IMREAD_GRAYSCALE);
    if (imgL.empty() || imgR.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다." << std::endl;
    }
    Mat Depthmap;
    depth->MonoDepthEstimate(imgL,imgR,calib_->GetCameraMatrix() , calib_->GetdistCoeffs(),Depthmap);

    delete depth;
    log_->Log(INFO,"Core Initialize Done");
}

Core::~Core()
{
    
}

