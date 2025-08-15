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

using namespace std::chrono_literals;

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
    pStorageManager_ = make_uptr<Manager::StorageManager>(self, log_);
    pRequestManager_ = make_uptr<Manager::RequestManager>(self, log_);
    pPathSyncManager_ = make_uptr<Manager::PathSyncManager>(self, _AMR_NUM_, log_);

    for (int i = 0; i < RobotArm::RobotArmNum; ++i)
    {
        RobotArm_Adapters_.emplace_back(make_uptr<Adapter::RobotArmAdapter>(self,i ,log_));
    }

    traffic_Planner_ = make_uptr<TrafficPlanner>(Commondefine::map, log_);
    
    // AMR 어댑터 생성
    for (int i = 0; i < _AMR_NUM_; ++i)
    {
        amr_adapters_.emplace_back(make_uptr<Adapter::AmrAdapter>(self, log_, i));
    }

    log_->Log(Log::LogLevel::INFO, "Core Initialize Done");
    
    return true;
}

bool Core::assignTask(int idx, Commondefine::AmrStep step)
{
    if(idx > amr_adapters_.size()) return false;

    switch (step)
    {
    case AmrStep::check_path_update:
        log_->Log(Log::LogLevel::INFO, "assignTask check_path_update");
        assignTask(std::bind(&Adapter::AmrAdapter::checkPathUpdate, amr_adapters_[idx].get()));
        break;

    case AmrStep::MoveTo_Storage:
        log_->Log(Log::LogLevel::INFO, "assignTask MoveTo_Storage");
        assignTask(std::bind(&Adapter::AmrAdapter::MoveToStorage, amr_adapters_[idx].get()));
        break;

    case AmrStep::MoveTo_charging_station:
        log_->Log(Log::LogLevel::INFO, "assignTask MoveTo_charging_station");
        assignTask(std::bind(&Adapter::AmrAdapter::MoveToChargingStation, amr_adapters_[idx].get()));
        break;

    case AmrStep::MoveTo_dst:
        log_->Log(Log::LogLevel::INFO, "assignTask MoveTo_dst");
        assignTask(std::bind(&Adapter::AmrAdapter::MoveTo, amr_adapters_[idx].get()));
        break;

    default:
        log_->Log(ERROR,"정의 되지 않은 RobotArm Step 입니다.");
        return false;
    }

    return true;
}

bool Core::assignTask(int idx, Commondefine::RobotArmStep step)
{   
    if(idx > RobotArm_Adapters_.size()) return false;

    switch (step)
    {
    case RobotArmStep::check_work_only_once:
        log_->Log(Log::LogLevel::INFO, "assignTask check_work_only_once");
        assignTask(std::bind(&Adapter::RobotArmAdapter::checkWorkOnlyOnce, RobotArm_Adapters_[idx].get()));
        break;
    
    case RobotArmStep::resolve_Request:
        log_->Log(Log::LogLevel::INFO, "assignTask resolve_Request");
        assignTask(std::bind(&Manager::StorageManager::resolveRequest, pStorageManager_.get()));
        break;
    
    case RobotArmStep::shelf_to_buffer:
        log_->Log(Log::LogLevel::INFO, "assignTask shelf_to_buffer");
        assignTask(std::bind(&Adapter::RobotArmAdapter::shelfToBuffer, RobotArm_Adapters_[idx].get()));
        break;

    case RobotArmStep::buffer_to_Amr:
        log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_Amr");
        assignTask(std::bind(&Adapter::RobotArmAdapter::bufferToAmr, RobotArm_Adapters_[idx].get()));
        break;

    case RobotArmStep::Amr_to_buffer:
        log_->Log(Log::LogLevel::INFO, "assignTask Amr_to_buffer");
        assignTask(std::bind(&Adapter::RobotArmAdapter::amrToBuffer, RobotArm_Adapters_[idx].get()));
        break;

    case RobotArmStep::buffer_to_shelf:
        log_->Log(Log::LogLevel::INFO, "assignTask buffer_to_shelf");
        assignTask(std::bind(&Adapter::RobotArmAdapter::bufferToshelf, RobotArm_Adapters_[idx].get()));
        break;

    default:
        log_->Log(ERROR,"정의 되지 않은 RobotArm Step 입니다.");
        return false;
    }

    return true;
}

bool Core::assignPath(Integrated::Task task)
{
    assignTask(task);

    return true;
}

void Core::assignWork(int amr, Commondefine::GUIRequest r)
{
    if(amr_adapters_.size() < amr) return;

    //1. 로봇에게 요청 들어온 정보를 할당한다.
    amr_adapters_[amr]->SetTaskInfo(r);

    //2. 창고로 실제 목적지 및 현재 스텝 설정
    amr_adapters_[amr]->SetCurrentDst(Commondefine::wpStorage);
    amr_adapters_[amr]->SetAmrStep(AmrStep::MoveTo_Storage);
    
    //3. 새로운 경로를 생성하기 위해 윈도우 오픈
    pPathSyncManager_->OpenSyncWindow();

    //4. 로봇에게 일을 할당 하기 때문에 BUSY 상태가 된다.
    amr_adapters_[amr]->SetAmrState(Commondefine::RobotState::BUSY);

    //5. 로봇팔에 명령 추가
    Commondefine::StorageRequest req;
    req.robot_id = RobotArm::RobotArm1;
    req.amr_id = amr;
    req.shoes = r.shoes_property;
    req.command = RobotArmStep::shelf_to_buffer;
    pStorageManager_->StorageRequest(req);

    //6. AMR의 창고 이동 명령
    assignTask(amr, AmrStep::check_path_update);

    return;
}

bool Core::ArmRequestMakeCall(Commondefine::RobotArm arm, int shelf_num, int robot_id, std::string action)
{
    auto iface = Interface_.lock();
    if(iface == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR,"RosInterface is nullptr");
        return false;
    }

    switch (arm)
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
        log_->Log(Log::LogLevel::ERROR, "수신된 pos의 갯수가 현재 AMR 갯수보다 많습니다.");
        return false;
    }

    size_t size = pos.size();
    for(size_t robot_id = 0 ; robot_id < size; ++robot_id)
    {
        amr_adapters_[robot_id]->SetCurrentPose(pos[robot_id]);

        if(amr_adapters_[robot_id]->GetAmrState() == RobotState::IDLE) continue;

#ifdef _USE_ASSUNG_TASK_
        assignTask(std::bind(&AmrAdapter::handleWaypointArrival,amr_adapters_[robot_id].get(), pos[robot_id]));
#else
        return amr_adapters_[robot_id]->handleWaypointArrival(pos[robot_id]);    
#endif
    }

    return true;
}

bool Core::ArmDoneCallback(ArmRequest request)
{
    if(request.robot_id > RobotArm_Adapters_.size()) return false;

    if(!request.success) return false;

    //작업 완료한 로봇 IDEL 상태로 변경하고 둘중 한명이 일이 끝났는지 설정
    RobotArm_Adapters_[request.robot_id]->setState(RobotState::IDLE);
    pStorageManager_->SetWorkOnlyOnce(true);

    //로봇팔 2번에 완료 됬다는 신호를 보내주면 공유 자원을 사용가능 하게 설정한다.
    if(request.robot_id == RobotArm::RobotArm2) 
    {
        pStorageManager_->setCriticalSection(true);


        OpenSyncWindow(); // 완료가 된 시점에 새로운 경로 생성을 요청하고, path가 업데이트 되도록 기다린다.

        //현재 세대에 참여하는 로봇의 갯수는 BUSY 상태의 로봇들이기 때문에 본인 까지 포함이 되어 있는 상태이다.
        //이때 handleWaypointArrival() 내부에서 도착 완료 여부를 카운트 하는데, 지금 함수에서는 이미 도착한 상태이기 때문에
        //그냥 바로 윈도우를 오픈 하고 
        ArriveAtSyncOnce(request.amr_id);
        
        //핑키에게 다음 번 주행 명령을 보낸다. 실제 목적지로 간다.
        assignTask(request.amr_id, AmrStep::check_path_update);
    }

    //수거 요청이 들어오면 수거 하도록 한다.
    if(request.action == "buffer_to_shelf")
    {
        Commondefine::StorageRequest storage;
        storage.robot_id = Commondefine::RobotArm::RobotArm1;
        storage.amr_id = request.amr_id;
        storage.command = RobotArmStep::buffer_to_shelf;
        storage.shoes = request.shoes;

        pStorageManager_->StorageRequest(storage);
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
                //로봇의 상태를 RETURN 으로 처리하고 충전 위치로 보낸다.
                amr_adapters_[i]->SetAmrState(RobotState::RETURN);
                amr_adapters_[i]->SetCurrentDst(Commondefine::wpChargingStation[i]);
                amr_adapters_[i]->SetAmrStep(Commondefine::AmrStep::MoveTo_charging_station);

                OpenSyncWindow(); // 완료가 된 시점에 새로운 경로 생성을 요청하고, path가 업데이트 되도록 기다린다.

                assignTask(i,Commondefine::AmrStep::check_path_update);

                log_->Log(Log::LogLevel::INFO, string("핑키가 고객ID: ") + to_string(customer_id) + "에게 배달 완료");
                           
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
        //만들어야 함 ^^
        return true;
    }
    
    else return false ;
}
       
bool Core::publishNavGoal(int idx, const Commondefine::Position wp)
{
    auto iface =  Interface_.lock();
    if (!iface) return false;

    iface->publishNavGoal(idx, wp);

    return true;
}

void Core::PlanPaths()
{
    vector<Commondefine::Position> starts;
    vector<Commondefine::Position> goals;
    std::vector<size_t> active_indices;
    active_indices.reserve(amr_adapters_.size());

    for(size_t i = 0 ; i < amr_adapters_.size(); ++i)
    {
        auto& amr = amr_adapters_[i];

        if(amr->GetAmrState() == RobotState::IDLE) continue;
        
        starts.push_back(amr->GetCurrentPoseToWp());
        goals.push_back(amr->GetCurrentDst());

        active_indices.push_back(i);
    }

    //실제 경로 작성
    auto paths = traffic_Planner_->planPaths(starts, goals);

    if(paths.empty() || paths.size() != active_indices.size())
    {
        log_->Log(ERROR,"path가 비어있거나, path의 크기와 실제 요청한 로봇의 갯수가 맞지 않습니다.");
        return;
    }

    for(size_t i = 0 ; i < paths.size(); ++i)
    {
        size_t idx = active_indices[i];
        amr_adapters_[idx]->updatePath(paths[i]);
    }
}

bool Core::SendPickupRequest(int idx)
{
    // 요청또한 스케줄링이 필요하기 때문에 StorageManager
    Commondefine::StorageRequest storage;
    storage.robot_id = RobotArm::RobotArm2;
    storage.amr_id = idx; 
    storage.command = RobotArmStep::buffer_to_Amr;
    storage.container = ContainerType::Shelf;

    pStorageManager_->StorageRequest(storage);

    return true;
}

void Core::assignPlanPaths()
{
    assignTask(std::bind(&core::Core::PlanPaths,this));
}

bool Core::waitNewPath(std::chrono::milliseconds ms)
{
    std::unique_lock lock(path_mtx_);

    return path_cv_.wait_for(lock, ms,[&](){ return !IsSyncOpen(); });
}

Commondefine::RobotState Core::GetAmrState(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return RobotState::INVALID;

    return amr_adapters_[idx]->GetAmrState();
}

void Core::UpdateBattery(int idx, float percent)
{
    if (idx < 0 || idx >= static_cast<int>(amr_adapters_.size())) return;

    amr_adapters_[idx]->SetBattery(percent);
}

int Core::GetAmrBattery(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return -1;

    return amr_adapters_[idx]->GetBattery();
}

int Core::GetAmrCustID(int idx)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return -1;
        
    return amr_adapters_[idx]->GetTaskInfo().customer_id;
}

void Core::SetTaskInfo(int idx, const Commondefine::GUIRequest& request)
{
    if (idx < 0 || idx >= amr_adapters_.size()) return;

    amr_adapters_[idx]->SetTaskInfo(request);
}

int Core::GetAmrVecSize()
{
    return amr_adapters_.size();
}

bool Core::setStorageRequest(Commondefine::StorageRequest& Request)
{
    if(Request.robot_id > RobotArm_Adapters_.size()) return false;

    RobotArm_Adapters_[Request.robot_id]->setStorageRequest(Request);

    return true;
}

void Core::assignBestRobotSelector()
{
    assignTask(std::bind(&Manager::RequestManager::BestRobotSelector,pRequestManager_.get()));
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

int Core::findEmptyStorage(Commondefine::ContainerType Container)
{
    return pStorageManager_->findEmptyStorage(Container);
}

bool Core::waitCriticalSection(std::chrono::milliseconds ms)
{
    return pStorageManager_->waitCriticalSection(ms);
}

int Core::CurrentActiveRobotCount()
{
    int count = 0;
    for(auto& p : amr_adapters_)
    {
        if(p->GetAmrState() == RobotState::BUSY) ++count;
    }

    return count;
}

//#define _USE_ASSIGN_
void Core::ReplanAndBroadcast()
{
#ifdef _USE_ASSIGN_
    assignPlanPaths();
#else
    PlanPaths();
#endif
   
}

bool Core::IsSyncOpen()
{
    return pPathSyncManager_->IsSyncOpen();
}
        
void Core::ArriveAtSyncOnce(int robot_id)
{
    pPathSyncManager_->ArriveAtSyncOnce(robot_id);
}

void Core::OpenSyncWindow()
{
    pPathSyncManager_->OpenSyncWindow();
}

bool Core::waitWorkOnlyOnce(std::chrono::milliseconds ms)
{
    return pStorageManager_->waitWorkOnlyOnce(ms);
}

void Core::setWorkOnlyOnce(bool flag)
{
    pStorageManager_->SetWorkOnlyOnce(flag);
}