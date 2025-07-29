#pragma once

#include "Integrated.hpp"
#include "RobotArm.hpp"
#include "Commondefine.hpp"
#include "Calibrator.hpp"
#include "DepthEstimate.hpp"
#include "RosInterface.hpp"

#include "opencv2/opencv.hpp"

namespace core
{
    class Core : public std::enable_shared_from_this<Core>
    {
    private:
        Log::Logger::s_ptr                                    log_;
        Integrated::u_ptr<RA::RobotArm>                       RobotArm_;
        Integrated::w_ptr<interface::RosInterface>            Interface_;
        Integrated::u_ptr<cv::VideoCapture>                   video_;
        Integrated::u_ptr<Calib::Calibrator>                  calib_;

    public:
        Core(Logger::s_ptr log , Integrated::w_ptr<interface::RosInterface> Interface);
        ~Core();
    };
};