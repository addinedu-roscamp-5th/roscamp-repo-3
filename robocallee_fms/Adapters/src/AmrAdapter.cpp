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
            << ", model=" << robot_task_info_.shoes_property.model
            << ", size=" << robot_task_info_.shoes_property.size
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