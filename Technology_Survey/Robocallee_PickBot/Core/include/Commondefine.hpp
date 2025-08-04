#pragma once
#include <cmath>

namespace Commondefine
{
    #define _ROS_NODE_NAME_ "robocallee_pickbot"

    #define _LOG_FILE_DIR_ "robocallee_pickbot_Log_file"

    #define _CALIB_FILE_DIR_ "calib_file/Calib.XML"

    #define JETCOBOT "/dev/ttyJETCOBOT"
    
    #define BAUDRATE 1000000

    #define _TIME_OUT_ 500

    #define _CAM_INDEX_ 0

    enum RobotArm_ID {RobotArm_1 = 0 , RobotArm_2 = 1};

    enum Params {a = 0, alpha, d, theta_offset};

    #define DH_PARAM 4

    const std::vector<std::array<double, 4>> mycobot280_dh_params =
    {
        //{   a,   alpha,            d, theta_offset }
        {   0.0,    M_PI_2,     131.22,         0.0 }, // joint 1
        {-110.4,        0.0,       0.0,     -M_PI_2 }, // joint 2
        { -96.0,        0.0,       0.0,         0.0 }, // joint 3
        {   0.0,     M_PI_2,      63.4,     -M_PI_2 }, // joint 4
        {   0.0,    -M_PI_2,     75.05,      M_PI_2 }, // joint 5
        {   0.0,        0.0,      45.6,         0.0 }  // joint 6
    };
};