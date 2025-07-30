#include "AmrAdapter.hpp"

using namespace Adapter;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const std::string& name)
    :Icore_(Icore),log_(log)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");
    task_info_.robot_id = name;
}

AmrAdapter::~AmrAdapter()
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 소멸");
}

void AmrAdapter::UpdateTaskInfo(const Commondefine::RequestInfo& request)
{
    // if (request.requester=="customer"){
    //     // task_info_.dest1 = 창고
    //     // task_info_.dest2 = 배달지
        
    // }
    // else if (request.requester=="employee"){
        // task_info_.dest1 = 수거함
        // task_info_.dest2 = 창고
        
    // }

    task_info_.shoes_property = request.shoes_proptery;
   
    task_info_.requester = request.requester;
    task_info_.customer_id = request.customer_id;

    
}

Commondefine::RobotTaskInfo& AmrAdapter::GetTaskInfo()
{
    return task_info_;
}