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
    pStorageManager_ = make_uptr<pStorageManager_>(self, log_);
    pRequestManager_ = make_uptr<RequestManager>(self, log_);

    for (int i = 0; i < RobotArm::RobotArmNum; ++i)
    {
        RobotArm_Adapters_.emplace_back(make_uptr<Adapter::RobotArmAdapter>(self,i ,log_));
    }

    occupancyGrid_ = make_uptr<OccupancyGrid>();
    occupancyGrid_->LoadOccupancyGrid(_YAML_PATH_,_YAML_FILE_);

    traffic_Planner_ = make_sptr<TrafficPlanner>(occupancyGrid_->GetBoolMap(), log_);
    
    // AMR 어댑터 생성
    for (int i = 0; i < _AMR_NUM_; ++i)
    {
        amr_adapters_.emplace_back(make_uptr<Adapter::AmrAdapter>(self, log_, i));
    }

    log_->Log(Log::LogLevel::INFO, "Core Initialize Done");
    
    return true;
}

bool Core::SetAmrNextStep(int idx, Commondefine::AmrStep step)
{
    if(amr_adapters_.size() < idx)
    {
        log_->Log(Log::LogLevel::INFO, "Index가 넘었습니다.");
        return false;
    } 

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

bool Core::assignTask(int idx, Commondefine::AmrStep step)
{
    if(idx > amr_adapters_.size()) return false;
    auto arm = amr_adapters_[idx];

    switch (step)
    {
    case constant expression:
        /* code */
        break;
    


    case AmrStep::MoveTo_charging_station:
    
        break;

    default:
        log_->Log(ERROR,"정의 되지 않은 RobotArm Step 입니다.")
        return false;
    }

    return true;
}

bool Core::assignTask(Commondefine::RobotArmStep step)
{   
    switch (step)
    {
    case check_critical_section:
        log_->Log(Log::LogLevel::INFO, "assignTask check_critical_section");
        assignTask(std::bind(&Manager::StorageManager::checkCriticalSection, pStorageManager_.get()));
        break;
    
    case shelf_to_buffer:
        log_->Log(Log::LogLevel::INFO, "assignTask shelf_to_buffer");
        assignTask(std::bind(&Adapter::RobotArmAdapter::shelfToBuffer, amr_adapters_[RobotArm::RobotArm1].get()));
        break;

    case buffer_to_Amr:
        log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_Amr");
        assignTask(std::bind(&Adapter::RobotArmAdapter::bufferToAmr, amr_adapters_[RobotArm::RobotArm2].get()));
        break;

    case Amr_to_buffer:
        log_->Log(Log::LogLevel::INFO, "assignTask Amr_to_buffer");
        assignTask(std::bind(&Adapter::RobotArmAdapter::amrToBuffer, amr_adapters_[RobotArm::RobotArm2].get()));
        break;

    case buffer_to_shelf:
        log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_shelf");
        assignTask(std::bind(&Adapter::RobotArmAdapter::bufferToAmr, amr_adapters_[RobotArm::RobotArm1].get()));
        break;

    default:
        log_->Log(ERROR,"정의 되지 않은 RobotArm Step 입니다.")
        return false;
    }

    return true;
}

bool Core::ArmRequestMakeCall(Commondefine::RobotArm arm, int shelf_num, int robot_id, std::string action)
{
    auto iface = Interface_.lock();
    if(iface == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR,"RosInterface가 유효하지 않습니다!");
        return false;
    }

    switch (arm_num)
    {
    case RobotArm::RobotArm1:
        iface->arm1_send_request(shelf_num, robot_id, action);
        break;

    case RobotArm::RobotArm2:
        iface->arm2_send_request(robot_id, action);
        break;
    
    default:
        log_->Log(Log::LogLevel::ERROR,"Arm Index가 유효하지 않습니다!");
        return false;
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

        if(amr_adapters_[robot_id]->GetAmrState() == RobotState::IDLE) continue;

#ifdef _USE_ASSUNG_TASK_
        assignTask(std::bind(&AmrAdapter::handleWaypointArrival,amr_adapters_[robot_id].get(), pos[robot_id]));
#else
        return amr_adapters_[robot_id]->handleWaypointArrival(pos[robot_id]);    
#endif
    }

    return true;
}

bool Core::ArmDoneCallback(int id, std::string action, bool success)
{
    if(id > RobotArm_Adapters_.size()) return false;

    if(success)
    {
        RobotArm_Adapters_[id]->setState(RobotState::IDLE);
    }

    if(action == "buffer_to_shelf")
    {
        Commondefine::StorageRequest storage;
        storage.robot_id = Commondefine::RobotArm::RobotArm1;
        storage.command = RobotArmStep::buffer_to_shelf;

        pStorageManager_->StorageRequest(storage)
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

        pRequestManager_->BestRobotSelector();

        return wait_list;
    }
    else
    {   //error
        return -1;
    }
}

bool Core::DoneCallback(const std::string& requester, const int& customer_id)
{
    log_->Log(Log::LogLevel::INFO, string("Done received from: ") + requester);
    
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
        // 관리자가 수거함에서 누르기 때문에 수거함 -> 창고
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
        if(amr->GetTaskInfo().GetAmrState() == RobotState::IDLE) continue;
        
        amr->WaitUntilWaypointOccupied();
        
        starts.push_back(amr->GetCurrentPosition());
        goals.push_back(amr->GetDestPosition());
    }

    auto paths = traffic_Planner_->planPaths(starts, goals);

    size_t size = amr_adapters_.size();
    
    for(size_t i = 0 ; i < size; ++i)
    {
        if(amr_adapters_[i]->GetAmrState() == RobotState::IDLE) continue;

        amr_adapters_[i]->updatePath(paths[i]);
    }

    assignNewAmr_ = false;
    path_cv_.notify_all();
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

    return amr_adapters_[idx]->GetAmrState();
}

int Core::GetAmrBattery(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return -1;

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

void Core::assignWork(int amr, Commondefine::GUIRequest r)
{
    if(amr_adapters_ < amr) return;

    //1. 로봇에게 일을 할당 하기 때문에 BUSY 상태가 된다.
    amr_adapters_[amr]->SetAmrState(Commondefine::RobotState::BUSY);

    //2. 로봇에게 요청 들어온 정보를 할당한다.
    SetTaskInfo(amr, r);
    
    //3. 새로운 로봇이 추가됬다는 flag 추가
    SetAssignNewAmr(true);

    //5. 로봇팔에 명령 추가
    StorageRequest req;
    req.robot_id = amr;
    req.shoes = r.shoes_property;
    req.command = RobotArmStep::check_critical_section;

    //6. 로봇팔의 명령 추가
    pStorageManager_->StorageRequest(req);
    pStorageManager_->waitCriticalSection();

    SetAmrNextStep(best_amr, Commondefine::AmrStep::MoveTo_dest1);

    // 베스트 핑키 구분을 해야하는 이유가 있다. 로봇팔이 작업을 안할떄 불리우면 안된다. 수거 요청 불렸을때, 수거위치 까지 가는것
    // if (req.requester == "customer")
    // {
    //     Commondefine::shoesproperty shoe_info = req.shoes_property;
    //     core->SetRobotArmNextStep(Commondefine::RobotArmStep::shelf_to_buffer , dummy_shoe , best_amr );
    //     log_->Log(Log::LogLevel::INFO, "로봇팔 작업 지정: " + shoe_info.model + ", " + shoe_info.color + ", " + std::to_string(shoe_info.size) + ", 핑키 번호: " + std::to_string(best_amr));
    // }

    return;
}

bool setStorageRequest(Commondefine::StorageRequest& Request)
{
    if(Request.robot_id > RobotArm_Adapters_.size()) return false;

    RobotArm_Adapters_[Request.robot_id]->setStorageRequest(Request);

    return true;
}

bool Core::findStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request)
{
    const int err = -1; 

    int index = pStorageManager_->findStorage(Container , Request.shoes);

    if(Request.containerIndex == err) return false;
    Request.container = Container;
    Request.containerIndex = index;

    return true;
}

bool Core::setStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest Request)
{
    return pStorageManager_->setStorage(Container,Request);
}

bool Core::getStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request)
{
    return pStorageManager_->getStorage(Container,Request);
}

int findEmptyStorage(Commondefine::ContainerType Container)
{
    return pStorageManager_->findEmptyStorage(Container);
}