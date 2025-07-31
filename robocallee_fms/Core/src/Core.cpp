#include "Core.hpp"

using namespace core;
using namespace task;
using namespace Integrated;
using namespace Commondefine;
using namespace Adapter;
using namespace std;
using namespace Manager;

Core::Core(Logger::s_ptr log, interface::RosInterface::s_ptr Interface)
    :log_(log) , Interface_(Interface)
{
    log_->Log(Log::LogLevel::INFO,"Core 객체 생성");

  

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
    
    pRobotArmAdapter_ = make_uptr<RobotArmAdapter>(self, log_);

    pRequestManager_ = make_uptr<RequestManager>(self, log_);

    // arm adapter 객체 생성 및 vector에 추가
    for(int i = 0; i < _AMR_NUM_ ; ++i)
    {
        std::string name = "AMR" + std::to_string(i+1);
        amr_adapters_.emplace_back(make_uptr<Adapter::AmrAdapter>(self, log_ , name));   
    }

    log_->Log(Log::LogLevel::INFO,"Core Initialize Done");
    
    return true;
}


bool Core::SetAmrNextStep(int idx, Commondefine::AmrStep step)
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


// auto Core::assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>
bool Core::SetRobotArmNextStep(Commondefine::RobotArmStep step , Commondefine::shoesproperty shoe_info , int pinky_num )

{
    switch (step)
    {
    case 1: //shelf_to_buffer
        // assignTask(pRobotArmAdapter_-> arm1_shelf_to_buffer, shoe_info, pinky_num);
        log_->Log(Log::LogLevel::INFO, "assignTask 직전" );
        assignTask( [this, shoe_info, pinky_num]() {pRobotArmAdapter_->arm1_shelf_to_buffer(shoe_info, pinky_num); } );
        // assignTask(Adapter::RobotArmAdapter::arm1_shelf_to_buffer);
        
        break;

    case 2: //buffer_to_pinky
        break;

    case 3: //pinky_to_buffer
        break;


    case 4: //buffer_to_shelf
        break;
    

    default:
        break;
    }

    return true;
}


// bool Core::ArmRequestMakeCall(const Commondefine::shoesproperty shoe_info , int pinky_num){

//     Interface_.arm1_response_callback()



//     return true;
// }



bool Core::RequestCallback(const Commondefine::GUIRequest& request)
{
    log_->Log(Log::LogLevel::INFO, "Request received: " + request.shoes_property.model);

    if (pRequestManager_)
    {
        pRequestManager_->EnqueueRequest(request);
        // {
            // auto core = core_.lock();
            // core->SetAmrNextStep(best_pinky_selector);
        // }
        pRequestManager_->BestRobotSelector();



        return true;
    }
    else
    {
        return false;
    }
}
   

bool Core::DoneCallback(const std::string& requester, const int& customer_id)
{
    log_->Log(Log::LogLevel::INFO, "Done received from: " + requester);

    if (requester == "customer")
    {
        for (int i = 0; i < _AMR_NUM_; i++)
        {
            if (GetAmrState(i) == customer_id)
            {
                amr_adapters_[i]->SetAmrState(Commondefine::RobotState::IDLE);
                log_->Log(Log::LogLevel::INFO, "핑키가 고객ID: " + to_string(customer_id) + "에게 배달 완료");
                pRequestManager_->BestRobotSelector();
                return true;
            }
            log_->Log(Log::LogLevel::INFO, "고객ID: " + to_string(customer_id) + "의 배달을 지정받은 핑키 없음");
            //완료 버튼을 누른 고객의 작업을 지정받은 핑키가 없다
            return false;
        }        
    }

    else if (requester == "employee")
    {
        // SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest2);
        return true;
    }
    
    else return false;
}


Commondefine::RobotState Core::GetAmrState(int idx) 
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return Commondefine::RobotState::INVALID;
    
    return amr_adapters_[idx]->GetTaskInfo().robot_state;
}


int Core::GetAmrBattery(int idx) 
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return -1;
    
    return amr_adapters_[idx]->GetTaskInfo().battery;
}

int Core::GetAmrCustID(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return -1;
    
    return amr_adapters_[idx]->GetTaskInfo().customer_id;
}


int Core::GetAmrVecSize()
{
    return amr_adapters_.size();
}


void Core::SetTaskInfo(int idx, const Commondefine::GUIRequest& request)
{
    if(idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return;

    amr_adapters_[idx]->SetTaskInfo(request);

}

