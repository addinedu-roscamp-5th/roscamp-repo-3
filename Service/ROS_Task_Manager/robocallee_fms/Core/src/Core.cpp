#include "Core.hpp"
#include <sstream>
#include <iomanip>  // for ostringstream
using namespace core;
using namespace task;
using namespace Integrated;
using namespace Commondefine;
using namespace Adapter;
using namespace std;
using namespace Manager;
using namespace traffic;
using namespace OG;
Core::Core(Logger::s_ptr log, interface::RosInterface::w_ptr Interface)
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
    // occupancyGrid_ = make_uptr<OccupancyGrid>();
    // occupancyGrid_->LoadOccupancyGrid(_YAML_PATH_,_YAML_FILE_);
    traffic_Planner_ = make_sptr<TrafficPlanner>(Commondefine::map, log_);
    // AMR 어댑터 생성
    for (int i = 0; i < _AMR_NUM_; ++i)
    {
        // string name = "AMR" + to_string(i);
        amr_adapters_.emplace_back(make_uptr<Adapter::AmrAdapter>(self, log_, i));
    }
    log_->Log(Log::LogLevel::INFO, "Core Initialize Done");
    return true;
}
bool Core::SetAmrNextStep(int idx, Commondefine::AmrStep step)
{
    assignNewAmr_ = true;
    PlanPaths();
    switch (step)
    {
    case Commondefine::MoveTo_dest1:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest1,
                             amr_adapters_[idx].get(), idx));
        break;
    case Commondefine::MoveTo_dest2:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest2,
                             amr_adapters_[idx].get(), idx));
        break;
    case Commondefine::MoveTo_dest3:
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo_dest3,
                             amr_adapters_[idx].get(), idx));
        break;
    default:
        break;
    }
    return true;
}
bool Core::SetRobotArmNextStep(Commondefine::RobotArmStep step, Commondefine::shoesproperty shoes, int robot_id)
{
    log_->Log(Log::LogLevel::INFO, "assignTask 직전 step은" + std::to_string(step));
    switch (step)
    {
        case RobotArmStep::shelf_to_buffer:
            log_->Log(Log::LogLevel::INFO, "assignTask shelf_to_buffer");
            assignTask(std::bind(&Adapter::RobotArmAdapter::arm1_shelf_to_buffer, pRobotArmAdapter_.get(), shoes, robot_id));
            break;
        case RobotArmStep::buffer_to_pinky:
            log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_pinky");
            assignTask(std::bind(&Adapter::RobotArmAdapter::arm2_buffer_to_pinky, pRobotArmAdapter_.get(), robot_id));
            break;
        case RobotArmStep::pinky_to_buffer:
            log_->Log(Log::LogLevel::INFO, "assignTask pinky_to_buffer");
            assignTask(std::bind(&Adapter::RobotArmAdapter::arm2_pinky_to_buffer, pRobotArmAdapter_.get(), robot_id));
            break;
        case RobotArmStep::buffer_to_shelf:
            log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_shelf");
            assignTask(std::bind(&Adapter::RobotArmAdapter::arm1_buffer_to_shelf, pRobotArmAdapter_.get(), robot_id));
            break;
        default:
            break;
    }
    return true;
}
bool Core::ArmRequestMakeCall(int arm_num, int shelf_num, int robot_id, std::string action)
{
    if (arm_num == 1) {
        if (auto iface = Interface_.lock()) {
            iface->arm1_send_request(shelf_num, robot_id, action);
        } else {
            log_->Log(Log::LogLevel::ERROR,
                      "RosInterface가 유효하지 않습니다!");
        }
    }
    else if (arm_num == 2) {
        if (auto iface = Interface_.lock()) {
            iface->arm2_send_request(robot_id, action);
        } else {
            log_->Log(Log::LogLevel::ERROR,
                      "RosInterface가 유효하지 않습니다!");
        }
    }
    return true;
}
#define _USE_ASSUNG_TASK_
bool Core::PoseCallback(const std::vector<Commondefine::pose2f> &pos)
{
    if(pos.size() > amr_adapters_.size())
    {
        log_->Log(Log::LogLevel::ERROR, "Pinky pos is not invalid");
        return false;
    }
    size_t size = pos.size();
    for(size_t robot_id = 0 ; robot_id < size; ++robot_id)
    {
        amr_adapters_[robot_id]->SetCurrentPosition(pos[robot_id]);
        if(amr_adapters_[robot_id]->GetTaskInfo().robot_state == RobotState::IDLE) continue;
#ifdef _USE_ASSUNG_TASK_
        assignTask(std::bind(&AmrAdapter::handleWaypointArrival,amr_adapters_[robot_id].get(), pos[robot_id]));
#else
        return amr_adapters_[robot_id]->handleWaypointArrival(pos[robot_id]);
#endif
    }
    return true;
}
int Core::RequestCallback(const Commondefine::GUIRequest& request)
{
    log_->Log(Log::LogLevel::INFO, "Request received: " + request.shoes_property.model);
    log_->Log(Log::LogLevel::INFO, "Request from: " + request.requester);
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
bool Core::DoneCallback(const std::string& requester, const int& customer_id)
{
    log_->Log(Log::LogLevel::INFO,
              string("Done received from: ") + requester);
    if (requester == "customer")
    {
        for (int i = 0; i < _AMR_NUM_; ++i)
        {
            if (GetAmrCustID(i) == customer_id)
            {
                amr_adapters_[i]->SetAmrState(RobotState::IDLE);
                log_->Log(Log::LogLevel::INFO, string("핑키가 고객ID: ") + to_string(customer_id) + "에게 배달 완료");
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
bool Core::publishNavGoal(int idx, const Commondefine::Position wp)
{
    auto iface = Interface_.lock();
    if (!iface) return false;
    iface->publishNavGoal(idx, wp);
    return true;
}
void Core::PlanPaths()
{
    vector<Commondefine::Position> starts;
    vector<Commondefine::Position> goals;
    for(const auto& amr : amr_adapters_)
    {
        if(amr->GetTaskInfo().robot_state == RobotState::IDLE) continue;
        amr->WaitUntilWaypointOccupied();
        starts.push_back(amr->GetCurrentPosition());
        goals.push_back(amr->GetDestPosition());
    }
    auto paths = traffic_Planner_->planPaths(starts, goals);
    
    size_t size = amr_adapters_.size();
    for(size_t i = 0 ; i < size; ++i)
    {
        if(amr_adapters_[i]->GetTaskInfo().robot_state == RobotState::IDLE) continue;
        amr_adapters_[i]->updatePath(paths[i]);
        assignNewAmr_ = false;
        path_cv_.notify_all();
        return;
    }
    auto iface = Interface_.lock();
    if (!iface)
    {
        log_->Log(Log::LogLevel::ERROR, "PlanPaths: Interface lock fail");
        assignNewAmr_ = false;
        path_cv_.notify_all();
        return;
    }
}
void Core::waitNewPath()
{
    if(!assignNewAmr_) return;
    std::unique_lock lock(path_mtx_);
    path_cv_.wait(lock,[&]()
    {
        return !assignNewAmr_;
    });
}
Commondefine::RobotState Core::GetAmrState(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return RobotState::INVALID;
    return amr_adapters_[idx]->GetTaskInfo().robot_state;
}
void Core::UpdateBattery(int idx, float percent)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return;
    std::lock_guard<std::mutex> lk(battery_mtx_);
    amr_adapters_[idx]->GetTaskInfo().battery = percent;
    log_->Log(Log::LogLevel::INFO, (std::ostringstream{} << "[BAT] Core::UpdateBattery AMR" << idx
                                                        << " = " << std::fixed << std::setprecision(1)
                                                        << percent << "%").str());
}
float Core::GetAmrBattery(int idx)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size()))
        return 0.f;
    return amr_adapters_[idx]->GetTaskInfo().battery;
}
int Core::GetAmrCustID(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return -1;
    return amr_adapters_[idx]->GetTaskInfo().customer_id;
}
int Core::GetAmrVecSize()
{
    return amr_adapters_.size();
}
void Core::SetTaskInfo(int idx, const Commondefine::GUIRequest& request)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return;
    amr_adapters_[idx]->SetTaskInfo(request);
}
void Core::assignWork(int amr)
{
    return;
}
bool Core::UpdateShelfInfo(Commondefine::shoesproperty incoming_shoe, int shelf_num)
{
    return true;
}
// void Core::SendTestGoal(int robot_id, int cell_x, int cell_y, float yaw_rad)
// {
//     int idx = robot_id - 1;
//     if (idx < 0 || idx >= GetAmrVecSize()) {
//         log_->Log(Log::LogLevel::ERROR, "SendTestGoal: invalid robot id " + std::to_string(robot_id));
//         return;
//     }
//     amr_adapters_[robot_id]->SetAmrState(RobotState::BUSY);
//     Commondefine::Position wp{};
//     wp.x = cell_x;  // 1-based grid
//     wp.y = cell_y;  // 1-based grid
//     wp.yaw = yaw_rad;
//     publishNavGoal(idx, wp);
//     log_->Log(Log::LogLevel::INFO,
//               "SendTestGoal -> robot#" + std::to_string(robot_id) +
//               " grid(" + std::to_string(wp.x) + "," + std::to_string(wp.y) +
//               "), yaw=" + std::to_string(wp.yaw));
// }
// void Core::SendTestGoal(int robot_id, const Commondefine::Position wp)
// {
//     int idx = robot_id - 1;
//     if (idx < 0 || idx >= GetAmrVecSize()) {
//         log_->Log(Log::LogLevel::ERROR, "SendTestGoal: invalid robot id " + std::to_string(robot_id));
//         return;
//     }
//     amr_adapters_[idx]->robot_task_info_.dest.x  = static_cast<float>(wp.x);
//     amr_adapters_[idx]->robot_task_info_.dest.y  = static_cast<float>(wp.y);
//     PlanPaths();
//     publishNavGoal(idx, wp);
//     log_->Log(Log::LogLevel::INFO,
//               "SendTestGoal(Pos) -> robot#" + std::to_string(robot_id) +
//               " grid(" + std::to_string(wp.x) + "," + std::to_string(wp.y) +
//               "), \yaw=" + std::to_string(wp.yaw));
// }