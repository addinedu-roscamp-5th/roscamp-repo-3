#include "Core.hpp"

using namespace core;
using namespace task;
using namespace Integrated;
using namespace Commondefine;
using namespace Adapter;
using namespace std;

Core::Core(Logger::s_ptr log, interface::RosInterface::s_ptr Interface)
    :log_(log) , Interface_(Interface)
{
    log_->Log(Log::LogLevel::INFO,"Core 객체 생성");

    Integrated::w_ptr<core::ICore> icore
    
    for (int i=0; i<Commondefine::_AMR_NUM_; ++i){
        auto adapter = u_ptr<Adapter::AmrAdapter>(icore, log_)

        std::string name = "amr" + std::to_string(i+1);
        
        // double battery = 100-i*20
        adapter->GetTaskInfo().robot_id = name;
        // adapter->GetTaskInfo().battery = battery;

        amr_adapters_.emplace_back(std::move(adapter));
           
    }

    log_->Log(Log::LogLevel::INFO,"Adapter 3개 객체 생성");
}

Core::~Core()
{
    log_->Log(Log::LogLevel::INFO,"Core 정상 종료");
}

bool Core::Initialize()
{
    //this 객체 shared_ptr 선언
    auto self = shared_from_this();

    pdispatcher_ = make_uptr<Dispatcher>(_MAX_EXECUTOR_NUM_, log_);

    pAmrAdapter_ = make_uptr<AmrAdapter>(self, log_);

    pRobotArmAdapter_ = make_uptr<RobotArmAdapter>(self, log_);

    log_->Log(Log::LogLevel::INFO,"Core Initialize Done");
    
    return true;
}


bool Core::SetAmrNextStep(Commondefine::AmrStep step)
{
    switch (step)
    {
    case AmrStep_num:
        break;
    
    default:
        break;
    }

    return true;
}

bool Core::SetRobotArmNextStep(Commondefine::RobotArmStep step)
{
    switch (step)
    {
    case RobotArmStep_num:
        break;
    
    default:
        break;
    }

    return true;
}

//////////
bool Core::RequestCallback(const Commondefine::GUIRequest& request)
{
    log_->Log(Log::LogLevel::INFO, "Request received: " + request.shoes_property.model);

    if (pRequestManager_)
    {
        pRequestManager_->EnqueueRequest(request);
        return true;
        // {
            // auto core = core_.lock();
            // core->SetAmrNextStep(best_pinky_selector);
        // }
    }
    else
    {
        return false;
    }
}
                    
bool Core::DoneCallback(const std::string& requester)
{
    log_->Log(Log::LogLevel::INFO, "Done received from: " + requester);

    if (requester == "customer")
    {
        // taskinfo->robot_state = IDLE;
        // addTask(best_pinky_selector());
        return true;
    }

    else if (requester == "employee")
    {
        // addTask(MoveTo_dest2(dest2, pinky_id));
        return true;
    }
    
    else return false;
}

std::vector<Adapter::AmrAdapter::u_ptr>& Core::GetAmrAdapters()
{
    return amr_adapters_;
}