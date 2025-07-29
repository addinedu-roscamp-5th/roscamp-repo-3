#pragma once

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
};