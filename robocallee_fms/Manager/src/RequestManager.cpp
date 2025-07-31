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
    std::lock_guard<std::mutex> lock(mtx_);
    request_queue_.push(r);
    log_->Log(Log::LogLevel::INFO, "EnqueueRequest 완료");
}

void RequestManager::PopRequest(Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (request_queue_.empty()) return;
    
    r = request_queue_.front();
    request_queue_.pop();

}

void RequestManager::BestRobotSelector()
{
    if (auto core = Icore_.lock()){

        Commondefine::GUIRequest req;

        int amrs_num = core->GetAmrVecSize();
        int best_amr = -1;
        int max_battery = -1;

        for (int i=0; i<amrs_num; i++)
        {
            Commondefine::RobotState status = core->GetAmrState(i);
            int battery = core->GetAmrBattery(i);

            if (status == Commondefine::RobotState::IDLE)
            {
                if (battery>max_battery){
                    max_battery = battery;
                    best_amr = i+1;
                }
            }
        }

        if (best_amr == -1){
            log_->Log(Log::LogLevel::INFO, "IDLE인 AMR 없음");
            return;
        }

        log_->Log(Log::INFO, "선택된 AMR: AMR" + best_amr);

        //밀린 작업 없음
        if (request_queue_.empty()){
            core->SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest3);
            return;
        }

        //요청 밀린 작업 있음
        PopRequest(req);
        core->SetTaskInfo(best_amr, req);

        // addTask(MoveTo_dest1(dest1, pinky_id));
        // addTask(로봇팔1_상차(신발정보, pinky_id));
            
        return;
        
    }

}