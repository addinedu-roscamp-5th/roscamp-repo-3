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

        for (int i=0; i<amrs_num; i++){
            std::string status = core->GetAmrState(i);
            if (status=="IDLE"){
                
            }
        }
        
    }

}

void UpdateAmrTask(int index)
{
    if (auto core = icore_.lock())
    {
        std::string status = core->GetAmrStatus(index);
        log_->Log(Log::INFO, "Current Amr[" + std::to_string(index) + "] status: " + status);

        core->SetAmrState(index, "BUSY");
    }
}