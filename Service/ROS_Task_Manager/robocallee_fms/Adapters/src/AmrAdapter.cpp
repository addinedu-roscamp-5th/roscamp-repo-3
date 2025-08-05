#include "AmrAdapter.hpp"

using namespace Adapter;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const std::string& name)
    :Icore_(Icore),log_(log)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");

    robot_task_info_.robot_id = name;
}

AmrAdapter::~AmrAdapter()
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 소멸");
}


Commondefine::RobotTaskInfo& AmrAdapter::GetTaskInfo()
{
    return robot_task_info_;
}

void AmrAdapter::SetTaskInfo(const Commondefine::GUIRequest& request)
{
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // if (request.requester=="customer"){
        //     robot_task_info_.dest1 = 창고
        //     robot_task_info_.dest2 = 배달지
            
        // }
        // else if (request.requester=="employee"){
        //     robot_task_info_.dest1 = 수거함
        //     robot_task_info_.dest2 = 창고
            
        // }

        robot_task_info_.robot_state = Commondefine::RobotState::BUSY;
        robot_task_info_.shoes_property = request.shoes_property;
    
        robot_task_info_.requester = request.requester;
        robot_task_info_.customer_id = request.customer_id;

        std::ostringstream oss;
        oss << ",color=" << robot_task_info_.shoes_property.color
            << ",model=" << robot_task_info_.shoes_property.model
            << ",size=" << robot_task_info_.shoes_property.size
            << ",pinky_id=" << robot_task_info_.robot_id;

        log_->Log(Log::LogLevel::INFO, oss.str());
        log_->Log(Log::LogLevel::INFO,"SetTaskInfo 완료");

    }

}

void AmrAdapter::SetAmrState(const Commondefine::RobotState& state)
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        
        robot_task_info_.robot_state = state;
    }
}

void AmrAdapter::onWaypointReached(int pinky_id)
{
    // if (current_waypoint_idx_ <static_cast<int>(waypoints_.size()) -1
    // {

    // }
}





void AmrAdapter::MoveTo_dest1()
{
//     auto core = core_.lock();
//     if (!core) return;

//     int pink_id = robots_info_.robot_id;

//     Commondefine::pose2d start = robots_info_.current_position;
//     Commondefine::pose2d goal  = robots_info_.dest1;

// // how to go from start to goal ? To core 
//     auto path = core->requestPath(pink_id, start, goal);

//     log_->Log(Log::LogLevel::INFO,
//               "AMR[%d] MoveTo_dest1: 경로 생성 후 Core에 퍼블리시 요청 (%zu 포인트)",
//               robotId, path.size())

//     core->SetAmrNextStep(pink_id, Commondefine::AmrStep::Load);


}

void AmrAdapter::MoveTo_dest2()
{
    // auto core = core_.lock();
    // if (!core) return;

    // int pink_id = robots_info_.robot_id;

    // auto path = core->requestPath(pink_id, start, goal);

    // core->SetAmrNextStep(pink_id, Commondefine::AmrStep::Unload);
    
}

void AmrAdapter::MoveTo_dest3()
{
    // auto core = core_.lock();
    // if(!core) return;

    // int pinky_id = robots_info_.robot_id;

    // auto path = core->requestPath(pink_id, start, goal);

    // core->SetAmrNextStep(pinky_id, Commondefine::AmrStep::Tocharge);
}