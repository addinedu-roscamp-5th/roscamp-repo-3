#pragma once
#include "Integrated.hpp"

namespace Commondefine
{
    #define _LOG_FILE_DIR_ "../../Log_file"

    #define _ROS_NODE_NAME_ std::string("robocallee_fms")

    #define _MAX_EXECUTOR_NUM_ 5

    #define _AMR_NUM_ 3

    #define _ARRIVAL_TOLERANCE_ 0.05f

    #define _MAP_RESOLUTION_ 0.1f

    #define _ARM_BUFFER_ 4

    #define _ARM_SHELF_ 9

    enum AmrStep {check_path_update = 0, MoveTo_Storage, MoveTo_dst, MoveTo_charging_station, AmrStep_num};

    enum RobotArmStep {check_work_only_once = 0,  resolve_Request , shelf_to_buffer, buffer_to_Amr, Amr_to_buffer, buffer_to_shelf, RobotArmStep_num };

    enum RobotState {IDLE = 0, BUSY, RETURN, STOP , INVALID};

    enum RobotArm { RobotArm1 = 0 ,RobotArm2 , RobotArmNum};

    enum ContainerType { Buffer, Shelf };

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
        pose2f                 dest2;
        int                    customer_id;
        std::string            action;
    }GUIRequest;

    typedef struct StorageRequest
    {
        shoesproperty shoes;
        ContainerType container;
        int containerIndex;
        int robot_id;
        int amr_id;
        RobotArmStep command;
    }StorageRequest;

    typedef struct ArmRequest
    {
        int robot_id;
        int amr_id;
        std::string action;
        shoesproperty shoes;
        bool success;
    }ArmRequest;

    typedef struct Position
    {
        int x;
        int y;
        double yaw;

        bool operator==(const Position& other) const { return x == other.x && y == other.y && std::abs(yaw - other.yaw) < 1e-6;}

    }Position;

    typedef struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    }Quaternion;

    typedef struct Constraint
    {
        int agent;
        int timestep;
        std::vector<Position> loc;  // size=1이면 vertex constraint, size=2이면 edge constraint
    
    }Constraint;

    typedef struct Conflict
    {
        int agent1;
        int agent2;
        int timestep;
        std::vector<Position> loc;  // 충돌 위치 (vertex 또는 edge)

    }Conflict;

    typedef struct Node
    {
        Position pos;
        int g_val;
        int h_val;
        int timestep;
        Node* parent;

        int f_val() const { return g_val + h_val; }
    }Node;

    typedef struct CBSNode
    {
        std::vector<std::vector<Position>> paths;
        std::vector<Constraint> constraints;
        int cost;
        std::vector<Conflict> conflicts;
        int id;

        bool operator>(const CBSNode& other) const { return cost > other.cost;}
    }CBSNode;

    typedef struct YAMLFile
    {
        std::string         Image_path_;
        double              resolution_;
        std::vector<double> origin_;
        bool                negate_;
        double              occ_thresh_;
        double              free_thresh_;

    } YAMLFile; 

    typedef struct RobotTaskInfo
    {
        int                                 robot_id;
        float                               battery = 100.f;
        shoesproperty                       shoes_property;
        pose2f                              dest;
        std::string                         requester;
        int                                 customer_id;
        
        RobotTaskInfo& operator=(const RobotTaskInfo& rhs)
        {
            if(this != &rhs)
            {
                this->robot_id = rhs.robot_id;
                this->battery = rhs.battery;
                this->shoes_property = rhs.shoes_property;
                this->dest = rhs.dest;
                this->requester = rhs.requester;
                this->customer_id = rhs.customer_id;
            }
            
            return *this;
        }
    }RobotTaskInfo;

    static double yaw(Commondefine::Position cur, Commondefine::Position next)
    {
        return std::atan2(next.y - cur.y, next.x - cur.x);
    }

    static Quaternion toQuaternion(Commondefine::Position p)
    {
        Commondefine::Quaternion q;
        q.x = static_cast<double>(p.x);
        q.y = static_cast<double>(p.y);
        q.z = std::sin(p.yaw / 2.0);
        q.w = std::cos(p.yaw / 2.0);
        
        return q;
    }

    static pose2f convertPositionToPose(Position p)
    {
        pose2f pose;
        pose.x = static_cast<float>((p.x - 1) * _MAP_RESOLUTION_ + _MAP_RESOLUTION_ / 2.0);
        pose.y = static_cast<float>((p.y - 1) * _MAP_RESOLUTION_ + _MAP_RESOLUTION_ / 2.0);

        return pose;
    }

    static Position convertPoseToPosition(pose2f pose)
    {
        Position p;
        p.x = static_cast<int>((pose.x - _MAP_RESOLUTION_ / 2.0) / _MAP_RESOLUTION_ + 1);
        p.y = static_cast<int>((pose.y - _MAP_RESOLUTION_ / 2.0) / _MAP_RESOLUTION_ + 1);

        return p;
    }


    static std::vector<std::vector<bool>> map
    {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    // static Position wpStorage = {5, 2, 0};

    static Position wpStorage = {6, 2, 0};

    static std::vector<Position> wpChargingStation{ {8,8,0},{8,6,0},{8,4,0}};
};



