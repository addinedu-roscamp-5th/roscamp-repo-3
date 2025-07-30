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