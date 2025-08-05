#include "Core.hpp"
#include <sstream>  // for ostringstream

using namespace core;
using namespace task;
using namespace Integrated;
using namespace Commondefine;
using namespace Adapter;
using namespace std;
using namespace Manager;
using namespace traffic;

Core::Core(Logger::s_ptr log, interface::RosInterface::s_ptr Interface)
    : log_(log), Interface_(Interface)
{
    log_->Log(Log::LogLevel::INFO, "Core 객체 생성");
    log_->Log(Log::LogLevel::INFO, "Adapter 3개 객체 생성");
}

Core::~Core()
{
    log_->Log(Log::LogLevel::INFO, "Core 정상 종료");
}

bool Core::Initialize()
{
    auto self = shared_from_this();

    pdispatcher_ = make_uptr<Dispatcher>(_MAX_EXECUTOR_NUM_, log_);
    pRobotArmAdapter_ = make_uptr<RobotArmAdapter>(self, log_);
    pRequestManager_ = make_uptr<RequestManager>(self, log_);
    traffic_solver_ = make_uptr<TrafficSolver>(self, log_);
    
    // AMR 어댑터 생성
    for (int i = 0; i < _AMR_NUM_; ++i) {
        string name = "AMR" + to_string(i+1);
        amr_adapters_.emplace_back(
            make_uptr<Adapter::AmrAdapter>(self, log_, name)
        );
    }\

    monitor_running_ = ture;
    assignTask([this]()) 
    {
        using namespace std::chorono_literals;
        while (monitor_running_)
        {
            for (int i = 0; i < _AMR_NUM_; ++i)
            {
                handleWaypointArrival(i);

            }
            std::this_thread::sleep_for(100ms);
        }
    }

    log_->Log(Log::LogLevel::INFO, "Core Initialize Done");
    return true;
}

bool Core::SetAmrNextStep(int idx, Commondefine::AmrStep step)
{
    switch (step)
    {
    case Commondefine::MoveTo_dest1:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest1,
                             pAmrAdapters_[pinky_id].get()));
    case Commondefine::arm2_buffer_to_pinky:
        assignTask(std::bind(&Adapter::AmrAdapter::arm2_buffer_to_pinky,
                             pAmrAdapters_[pinky_id].get()));
    case Commondefine::MoveTo_dest2:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest2,
                             pAmrAdapters_[pinky_id].get()));
    case Commondefine::amr2_pinky_to_buffer:
        assignTask(std::bind(&Adapter::AmrAdapter::amr2_pinky_to_buffer,
                             pAmrAdapters_[pinky_id].get()));
    case Commondefine::MoveTo_dest3:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest3,
                             pAmrAdapters_[pinky_id].get()));
    return true;
}

bool Core::SetRobotArmNextStep(Commondefine::RobotArmStep step,
                               Commondefine::shoesproperty shoe_info,
                               int pinky_num)
{
    // enum 출력을 위한 스트림 사용
    {
        ostringstream oss;
        oss << "assignTask 직전 step은 " << static_cast<int>(step);
        log_->Log(Log::LogLevel::INFO, oss.str());
    }

    switch (step) {
    case RobotArmStep::shelf_to_buffer:
        log_->Log(Log::LogLevel::INFO, "assignTask shelf_to_buffer");
        assignTask([this, shoe_info, pinky_num]() {
            pRobotArmAdapter_->arm1_shelf_to_buffer(shoe_info, pinky_num);
        });
        break;
    // case RobotArmStep::buffer_to_pinky:
    // case RobotArmStep::pinky_to_buffer:
    // case RobotArmStep::buffer_to_shelf:
    default:
        break;
    }
    return true;
}

bool Core::ArmRequestMakeCall(int arm_num, int shelf_num, int pinky_num)
{
    if (arm_num == 1) {
        if (auto iface = Interface_.lock()) {
            iface->arm1_send_request(shelf_num, pinky_num);
        } else {
            log_->Log(Log::LogLevel::ERROR,
                      "RosInterface가 유효하지 않습니다!");
        }
    }
    return true;
}

bool Core::PoseCallback(const Commondefine::pose2f& pos, int pinky_id)
{
    auto& task_info = amr_adapters_[pinky_id]->GetTaskInfo();
    task_info.current_position = pos;

    handleWaypointArrival(pinky_id);

    return true;
}


void Core::handleWaypointArrival(int pink_id) 
{

    if (GetAmrState(pinky_id) != Commondefine::RobotState::BUSY) 
    {
        return;
    }
    
    TrafficSolver solver(map, starts, goals);
    auto paths = solver.findSolution(true);



    paths[pink_id][index]
    const auto &wps = amr_adapters_[pink_id]->paths();
    int idx         = amr_adapters_[pink_id]->GetCurrentWaypointIndex();
    
    if (idx >= int(wps.size())) return;
    
    const auto &cur = amr_adapters_[pink_id]->GetTaskInfo().current_position;
    float dx = float(cur.x -wps[idx].x);
    float dy = float(cur.y -wps[idx].y);
    
    float dist = std::hypot(dx, dy);
    if (dist <=0.05f)
    {
        amr_adapters_[pinky_id]->incrementWaypointIndex();
        amr_adapters_[pinky_id]->onWaypointReached(pinky_id);
    }
}


int Core::RequestCallback(const Commondefine::GUIRequest& request)
{
    log_->Log(Log::LogLevel::INFO, "Request received: " + request.shoes_property.model);

    if (pRequestManager_)
    {
        int wait_list = pRequestManager_->EnqueueRequest(request);
        // if (wait_list>0)
        // {
        //     pRequestManager_->BestRobotSelector();

        //     return wait_list;
        // }

        pRequestManager_->BestRobotSelector();

        //대기자
        return wait_list;
    }
    else
    {   //error
        return -1;
    }
}

bool Core::DoneCallback(const std::string& requester,
                        const int& customer_id)
{
    log_->Log(Log::LogLevel::INFO,
              string("Done received from: ") + requester);
    if (requester == "customer") {
        for (int i = 0; i < _AMR_NUM_; ++i) {
            if (GetAmrState(i) == customer_id) {
                amr_adapters_[i]->SetAmrState(RobotState::IDLE);
                log_->Log(Log::LogLevel::INFO,
                          string("핑키가 고객ID: ") + to_string(customer_id) +
                          "에게 배달 완료");
                pRequestManager_->BestRobotSelector();
                log_->Log(Log::LogLevel::INFO, "DoneCallback true");
                return true;
            }

        }       
        log_->Log(Log::LogLevel::INFO, "고객ID: " + to_string(customer_id) + "의 배달을 지정받은 핑키 없음");
        //완료 버튼을 누른 고객의 작업을 지정받은 핑키가 없다
        return false;
    }

    else if (requester == "employee")
    {
        // SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest2);
        return true;
    }
    
    else return false ;
}

Commondefine::RobotState Core::GetAmrState(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size()))
        return RobotState::INVALID;
    return amr_adapters_[idx]->GetTaskInfo().robot_state;
}

int Core::GetAmrBattery(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size()))
        return -1;
    return amr_adapters_[idx]->GetTaskInfo().battery;
}

int Core::GetAmrCustID(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size()))
        return -1;
    return amr_adapters_[idx]->GetTaskInfo().customer_id;
}

int Core::GetAmrVecSize()
{
    return amr_adapters_.size();
}

void Core::SetTaskInfo(int idx, const Commondefine::GUIRequest& request)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return;
    amr_adapters_[idx]->SetTaskInfo(request);
}
