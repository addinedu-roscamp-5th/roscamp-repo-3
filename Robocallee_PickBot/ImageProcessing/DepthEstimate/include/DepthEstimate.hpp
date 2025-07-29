#include "Integrated.hpp"
#include "opencv2/opencv.hpp"

namespace depth
{
    class DepthEstimate
    {
    private:

    public:
        DepthEstimate();
        ~DepthEstimate();

        bool MonoDepthEstimate(
            const cv::Mat& left,
            const cv::Mat& right,
            const cv::Mat& K,
            const cv::Mat& D,
            cv::Mat& DepthMap);
    };
};