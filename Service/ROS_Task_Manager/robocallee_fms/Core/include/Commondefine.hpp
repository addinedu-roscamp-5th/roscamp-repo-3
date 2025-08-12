#pragma once
#include "Integrated.hpp"

namespace Commondefine
{
    #define _LOG_FILE_DIR_ "../../Log_file"

    #define _ROS_NODE_NAME_ std::string("robocallee_fms")

    #define _MAX_EXECUTOR_NUM_ 5

    #define _AMR_NUM_ 3

    #define _YAML_PATH_ "Map"

    #define _YAML_FILE_ "test_map.yaml"

    #define _ARRIVAL_TOLERANCE_ 0.05f

    #define _MAP_RESOLUTION_ 0.1d

    #define _ARM_BUFFER_ 4

    #define _ARM_SHELF_ 9

    enum AmrStep {MoveTo_Storage = 0, MoveTo_dst, MoveTo_charging_station, AmrStep_num};

    // enum RobotArmStep {RobotArmStep_num = 0};
    enum RobotArmStep {check_critical_section = 0, resolve_Request , shelf_to_buffer, buffer_to_Amr, Amr_to_buffer, buffer_to_shelf, RobotArmStep_num };

    enum RobotState {IDLE = 0, BUSY, RETURN, STOP , INVALID};

    enum RobotArm { RobotArm1 = 0 ,RobotArm2 , RobotArmNum};

    enum ContainerType { Buffer, Shelf };

    // enum Requester {CUSTOMER = 0, EMPLOYEE};

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
        RobotArmStep command;
    }StorageRequest;

    typedef struct Position
    {
        int x;
        int y;
        double yaw;

        Position(int x_, int y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
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
        int                                 battery = 100;
        shoesproperty                       shoes_property;
        pose2f                              current_position;
        pose2f                              start;
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
                this->current_position = rhs.current_position;
                this->current_position = rhs.current_position;
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
};



