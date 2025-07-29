#pragma once

namespace Commondefine
{
    #define _LOG_FILE_DIR_ "../../Log_file"

    #define _ROS_NODE_NAME_ std::string("robocallee_fms")

    #define _MAX_EXECUTOR_NUM_ 5

    #define _AMR_NUM_ 3

    enum AmrStep {AmrStep_num = 0};

    enum RobotArmStep {RobotArmStep_num = 0};

    enum Robot_state {Robot_state_Idle = 0 , Robot_state_busy, Robot_state_stop};

    typedef struct pose2f
    {
        float x;
        float y;
    }pose2f;

    typedef struct pose2d
    {
        double x;
        double y;
    }pose2d;

    typedef struct pose3f
    {
        float x;
        float y;
        float z;
    }pose3f;

    typedef struct pose3d
    {
        double x;
        double y;
        double z;
    }pose3d;

    typedef struct shoesproperty
    {
        int             size;
        std::string     model;
        std::string     color;
    }shoesproperty;

    typedef struct GUIRequest
    {
        std::string            requester;
        shoesproperty          shoes_property;
        pose2d                 dest2;
    }GUIRequest; 

    typedef struct RequestInfo
    {
        std::string         requester;
        pose2d              dest1;
        pose2d              dest2;
        std::string         customer_id;
        shoesproperty       shoes_proptery;
    }RequestQueue;

    typedef struct RobotTaskInfo
    {
        std::string                         robot_id;
        Robot_state                         robot_state;
        int                                 battery             = 100;
        shoesproperty                       shoes_property;
        pose2d                              current_position;
        pose2d                              dest1;
        pose2d                              dest2;
        pose2d                              dest3;
        std::string                         requester;
        int                                 customer_id;
        // RobotTaskInfo& operator=(const RobotTaskInfo& rhs)
        // {
        //     this->robot_id = rhs.robot_id
        //     this->robot_state = rhs.robot_state
        //     this->batterty = rhs.batterty
        //     this->shoe_model = rhs.shoe_model
        //     this->current_position = rhs.current_position
        //     this->dest1 = rhs.dest1
        //     this->dest2 = rhs.dest2
        //     this->dest3 = rhs.dest3
        //     this->requester = rhs.requester
        //     this->customer_id = rhs.customer_id
        //     return *this;
        // }
    }RobotTaskInfo;
};



