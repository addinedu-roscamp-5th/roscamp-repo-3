#pragma once

#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "opencv2/opencv.hpp"

namespace cap
{
    class ImageCapture
    {
    private:
        Log::Logger::s_ptr                                  log_;
        Integrated::u_ptr<cv::VideoCapture>                 videoCap_;
        cv::Mat                                             currentframe_;


        std::thread                                         capLoop_;
        std::mutex                                          capmtx_;
        bool                                                isRunning_;

        void ImageCaptureLoop();
    public:
        ImageCapture(Log::Logger::s_ptr log);
        ~ImageCapture();

        bool open(int index ,cv::VideoCaptureAPIs api);

        bool getCurrentframe(cv::Mat& frame);
        
    };
};