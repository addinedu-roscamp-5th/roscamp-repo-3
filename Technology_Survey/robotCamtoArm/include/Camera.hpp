#include <iostream>
#include <functional>

#include "opencv2/opencv.hpp"

namespace Cam
{
    typedef std::function<void(cv::Mat, int, int)> grabcallback;

    class Camera
    {
    public:
        Camera();
        virtual ~Camera();

    protected:
        virtual bool initCamera(grabcallback func) = 0;

        virtual bool Grab(cv::Mat& Image) = 0;

        virtual bool Grab(cv::Mat& Image) = 0;
    };
};
