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
    // request_queue_.push(r);
    // request_queue_.push(r);
    // request_queue_.push(r);
    // request_queue_.push(r);
    log_->Log(Log::LogLevel::INFO, "EnqueueRequest 완료");

    //대기번호(본인을 제외한 다른 요청자의 수)
    int wait_list = static_cast<int>(request_queue_.size()) - 1;
    log_->Log(Log::LogLevel::INFO, "wait_list: " + std::to_string(wait_list));
    return wait_list;
}

void RequestManager::PopRequest(Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (request_queue_.empty()) return;
    
    r = request_queue_.front();
    request_queue_.pop();

        log_->Log(Log::LogLevel::INFO, "PopRequest 완료");


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
                    best_amr = i;
                }
            }
        }

        if (best_amr == -1){
            log_->Log(Log::LogLevel::INFO, "IDLE인 AMR 없음");
            return;
        }

        log_->Log(Log::INFO, "선택된 AMR: AMR" + std::to_string(best_amr));
        // core->SetAssgignNewAmr(true);

        //밀린 작업 없음
        if (request_queue_.empty()){
            core->SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest3);
            log_->Log(Log::INFO, "밀린 작업 없음, MoveTo_dest3 호출");
            return;
        }

        //요청 밀린 작업 있음
        PopRequest(req);
        core->SetTaskInfo(best_amr, req);

        //AMR을 목적지 1로 이동
        core->SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest1);

        //로봇팔1에게 버퍼로 상자 이동 명령
        if (req.requester == "customer")
        {
            Commondefine::shoesproperty shoe_info = req.shoes_property;
            core->SetRobotArmNextStep(Commondefine::RobotArmStep::shelf_to_buffer , shoe_info , best_amr );
            log_->Log(Log::LogLevel::INFO, "로봇팔 작업 지정: " + shoe_info.model + ", " + shoe_info.color + ", " + std::to_string(shoe_info.size) + ", 핑키 번호: " + std::to_string(best_amr));
        }
        


        // 로봇팔 작업 지정. shelf_to_buffer 로 시작
        // Commondefine::shoesproperty shoe_info = req.shoes_property;
        // log_->Log(Log::LogLevel::INFO, "로봇팔 작업 지정: " + shoe_info.model + ", " + shoe_info.color + ", " + std::to_string(shoe_info.size) + ", 핑키 번호: " + std::to_string(best_amr));


        // if(req.requester == "customer")
        // {   
        //     log_->Log(Log::INFO, "customer 로부터의 요청");

        //     core->SetRobotArmNextStep(Commondefine::RobotArmStep::shelf_to_buffer , shoe_info , best_amr );
        // }
        // else if(req.requester == "employee")
        // {
        //     // 직원 요청에 대한 로봇팔 작업 지정
        //     core->SetRobotArmNextStep(Commondefine::RobotArmStep::pinky_to_buffer , shoe_info , best_amr  );
        // }
            
        return;
        
    }

}