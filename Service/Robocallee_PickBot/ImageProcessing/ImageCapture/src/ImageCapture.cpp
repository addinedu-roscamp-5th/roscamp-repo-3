#include "ImageCapture.hpp"

using namespace cap;
using namespace cv;
using namespace Commondefine;
using namespace Integrated;


ImageCapture::ImageCapture(Log::Logger::s_ptr log)
    : log_(log), isRunning_(true)
{
    log_->Log(INFO,"ImageCapture init..");

    videoCap_ = make_uptr<VideoCapture>();

    capLoop_ = std::thread(&ImageCapture::ImageCaptureLoop,this);
}

ImageCapture::~ImageCapture()
{
    isRunning_ = false;
    if(capLoop_.joinable())
    {
        capLoop_.join();
        log_->Log(INFO,"ImageCaptureLoop 정상 종료");
    }
}

bool ImageCapture::open(int index ,cv::VideoCaptureAPIs api)
{
    if(videoCap_ == nullptr) return false;

    return videoCap_->open(index, api);
}

bool ImageCapture::getCurrentframe(cv::Mat& frame)
{
    std::lock_guard lock(capmtx_);

    if(videoCap_ == nullptr || currentframe_.empty()) return false;

    frame = std::move(currentframe_);

    return true;
}

void ImageCapture::ImageCaptureLoop()
{
    int fps = videoCap_->get(cv::CAP_PROP_FPS);

    while(isRunning_)
    {
        {
            std::lock_guard lock(capmtx_);

            if(videoCap_ == nullptr) break;

            videoCap_->read(currentframe_);

            if(currentframe_.empty()) continue;
        }

        std::this_thread::sleep_for(std::chrono::seconds(fps));
    }

    log_->Log(INFO,"ImageCaptureLoop return");
    return;
}