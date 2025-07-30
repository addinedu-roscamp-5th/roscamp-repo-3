#include "RequestManager.hpp"

using namespace Manager;

RequestManager::RequestManager(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore), log_(log)
{
    log_->Log(Log::LogLevel::INFO, "RequestManager 객체 생성");
    
}

RequestManager::~RequestManager()
{
    log_->Log(Log::LogLevel::INFO, "RequestManager 객체 소멸");
}

void RequestManager::EnqueueRequest(const Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(request_mtx_);
    request_queue_.push(r);
}

bool RequestManager::PopRequest(Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(request_mtx_);
    if (request_queue_.empty()) return false;
    
    r = request_queue_.front();
    request_queue_.pop();
    return true;

}

void RequestManager::best_pinky_selector()
{
    if (auto core = Icore_.lock()){
        int amrs_num = core->GetAmrVecSize();

        for (int i=0; i<amrs_num; i++)
        {
            Commondefine::RobotState  status = core->GetAmrState(i);
            if (status == Commondefine::RobotState::IDLE)
            {
                
            }
        }
        
    }

}

void RequestManager::UpdateAmrTask(int index)
{
    if (auto core = Icore_.lock())
    {
        Commondefine::RobotState  status = core->GetAmrState(index);
        log_->Log(Log::INFO, "Current Amr[" + std::to_string(index) + "] status: IDEL");

        core->SetAmrState(index, Commondefine::RobotState::BUSY);
        log_->Log(Log::INFO, "Current Amr[" + std::to_string(index) + "] status: BUSY");
    }
}