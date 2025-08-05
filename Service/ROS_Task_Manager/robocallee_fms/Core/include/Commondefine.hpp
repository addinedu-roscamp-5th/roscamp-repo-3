#pragma once
#include "Integrated.hpp"

namespace Commondefine
{
    #define _LOG_FILE_DIR_ "../../Log_file"

    #define _ROS_NODE_NAME_ std::string("robocallee_fms")

    #define _MAX_EXECUTOR_NUM_ 5

    #define _AMR_NUM_ 3

    enum AmrStep {AmrStep_num = 0, MoveTo_dest1, Load, MoveTo_dest2, Unload, MoveTo_dest3};

    // enum RobotArmStep {RobotArmStep_num = 0};
    enum RobotArmStep {shelf_to_buffer=1, buffer_to_pinky, pinky_to_buffer, buffer_to_shelf };

    enum RobotState {IDLE = 0 , BUSY, STOP , INVALID};

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
    }GUIRequest; 

    typedef struct RobotTaskInfo
    {
        std::string                         robot_id;
        RobotState                          robot_state         = RobotState::IDLE;
        int                                 battery             = 100;
        shoesproperty                       shoes_property;
        pose2f                              current_position;
        pose2f                              dest1;
        pose2f                              dest2;
        pose2f                              dest3               = {1.0,0.4};
        std::string                         requester;
        int                                 customer_id;
        
        RobotTaskInfo& operator=(const RobotTaskInfo& rhs)
        {
            if(this != &rhs)
            {
                this->robot_id = rhs.robot_id;
                this->robot_state = rhs.robot_state;
                this->battery = rhs.battery;
                this->shoes_property = rhs.shoes_property;
                this->current_position = rhs.current_position;
                this->dest1 = rhs.dest1;
                this->dest2 = rhs.dest2;
                this->dest3 = rhs.dest3;
                this->requester = rhs.requester;
                this->customer_id = rhs.customer_id;
            }
            
            return *this;
        }
    }RobotTaskInfo;

    typedef struct Position
    {
        int x;
        int y;
        bool operator==(const Position& other) const { return x == other.x && y == other.y;}

    }Position;

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

};



