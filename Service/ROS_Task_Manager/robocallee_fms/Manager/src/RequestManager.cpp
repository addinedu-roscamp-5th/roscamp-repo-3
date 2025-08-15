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

int RequestManager::EnqueueRequest(const Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(mtx_);
    request_queue_.push(r);
    
    log_->Log(Log::LogLevel::INFO, "EnqueueRequest 완료");
    
    //대기번호(본인을 제외한 다른 요청자의 수)
    int wait_list = static_cast<int>(request_queue_.size()) - 1;
    log_->Log(Log::LogLevel::INFO, "wait_list: " + std::to_string(wait_list));

    return wait_list;
}

void RequestManager::PopRequest(Commondefine::GUIRequest& r)
{
    if (request_queue_.empty()) return;
    
    {
        std::lock_guard<std::mutex> lock(mtx_);
        r = request_queue_.front();
        request_queue_.pop();
    }

    log_->Log(Log::LogLevel::INFO, "PopRequest 완료");
}

void RequestManager::BestRobotSelector()
{
    const int err = -1;
    auto core = Icore_.lock();
    if (core == nullptr)
    {
        log_->Log(ERROR,"core Pointer 가 nillptr 입니다.");
        return;
    }
    
    Commondefine::GUIRequest req;

    int amrs_num = core->GetAmrVecSize();
    int best_amr = err;
    int max_battery = err;

    for (int i = 0; i < amrs_num; ++i)
    {
        Commondefine::RobotState status = core->GetAmrState(i);
        int battery = core->GetAmrBattery(i);

        if (status == Commondefine::RobotState::IDLE)
        {
            if (battery > max_battery)
            {
                max_battery = battery;
                best_amr = i;
            }
        }
    }

    if (best_amr == err)
    {
        log_->Log(Log::LogLevel::INFO, "IDLE인 AMR 없음");
        return;
    }

    log_->Log(Log::INFO, "선택된 AMR: AMR" + std::to_string(best_amr));

    // 요청 POP
    PopRequest(req);

    core->assignWork(1, req);
    // core->assignWork(best_amr, req);
    return;
}