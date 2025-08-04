#pragma once

#include "Integrated.hpp"
#include "opencv2/opencv.hpp"
#include "Geometry.hpp"

#include "objectDetector.hpp"
#include "ArUcoMakerDetector.hpp"

#include "Calibrator.hpp"

namespace depth
{
    class DepthEstimate
    {
    private:
        Integrated::w_ptr<OB::objectDetector>           detector_;
        Integrated::w_ptr<Calib::Calibrator>            calib_;

    public:
        DepthEstimate(
            Integrated::w_ptr<OB::objectDetector> detector,
            Integrated::w_ptr<Calib::Calibrator> calib
        );
        ~DepthEstimate();

        // double MonoDepthEstimate(
        //     cv::Mat& left,
        //     cv::Mat& right,
        //     cv::Mat& DepthMap,
        //     double baseline);

        double MonoDepthEstimate
        (
            Integrated::vec<cv::Point>& left,
            Integrated::vec<cv::Point>& right,
            Integrated::vec<cv::Point3d>& pointcloud,
            double baseline
        );

        double computeBaseLine(cv::Mat& T_base_grip1 , cv::Mat& T_base_grip2);
    };
};