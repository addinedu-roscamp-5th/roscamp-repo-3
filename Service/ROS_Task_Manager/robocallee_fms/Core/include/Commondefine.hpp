#pragma once
#include "Integrated.hpp"
namespace Commondefine
{
    #define _LOG_FILE_DIR_ "../../Log_file"
    #define _ROS_NODE_NAME_ std::string("robocallee_fms")
    #define _MAX_EXECUTOR_NUM_ 5
    #define _AMR_NUM_ 3
    #define _YAML_PATH_ "Map"
<<<<<<< Updated upstream
    #define _YAML_FILE_ "map001.yaml"
    #define _ARRIVAL_TOLERANCE_ 0.05f
    #define _MAP_RESOLUTION_ 0.1f
=======

    #define _YAML_FILE_ "map001.yaml"

    #define _ARRIVAL_TOLERANCE_ 0.05f

    #define _MAP_RESOLUTION_ 0.01f

>>>>>>> Stashed changes
    enum AmrStep {AmrStep_num = 0, MoveTo_dest1, MoveTo_dest2, MoveTo_dest3};
    // enum RobotArmStep {RobotArmStep_num = 0};
    enum RobotArmStep {shelf_to_buffer=1, buffer_to_pinky, pinky_to_buffer, buffer_to_shelf };
    enum RobotState {IDLE = 0 , BUSY, STOP , INVALID};
    enum RobotArm { RobotArm1 = 0 ,RobotArm2 , RobotArmNum};
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
        int robot_id;
    }StorageRequest;
    typedef struct Position
    {
        int x;
        int y;
        double yaw;
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
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
        RobotState                          robot_state         = RobotState::IDLE;
        float                               battery             = 100.f;
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
                this->robot_state = rhs.robot_state;
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
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
    static std::vector<std::vector<bool>> map
    {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };
<<<<<<< Updated upstream
};
=======
};



>>>>>>> Stashed changes
