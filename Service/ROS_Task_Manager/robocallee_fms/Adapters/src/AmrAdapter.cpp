#include "AmrAdapter.hpp"

using namespace Adapter;
using namespace Commondefine;
using namespace Integrated;
using namespace std::chrono_literals;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const int id)
    :Icore_(Icore), log_(log), isOccupyWaypoint_(false)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");

    robot_task_info_.robot_id = id;
    
    current_wp_idx_ = (0);
}

AmrAdapter::~AmrAdapter()
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 소멸");
}


Commondefine::RobotTaskInfo& AmrAdapter::GetTaskInfo()
{
    return robot_task_info_;
}

const Commondefine::RobotTaskInfo& AmrAdapter::GetTaskInfo() const
{
    return robot_task_info_;
}

void AmrAdapter::SetTaskInfo(const Commondefine::GUIRequest& request)
{
    {
        std::lock_guard<std::mutex> lock(Task_mtx_);
        robot_task_info_.shoes_property = request.shoes_property;
        robot_task_info_.requester = request.requester;
        robot_task_info_.customer_id = request.customer_id;
        robot_task_info_.dest = request.dest2;
    }

    std::string msg = "color = " + robot_task_info_.shoes_property.color +
    ", model = " + robot_task_info_.shoes_property.model + "size = " + std::to_string(robot_task_info_.shoes_property.size) +
    ", robot_id = " + std::to_string(robot_task_info_.robot_id);

    log_->Log(Log::LogLevel::INFO, msg);

    log_->Log(Log::LogLevel::INFO,"SetTaskInfo 완료");
}

void AmrAdapter::SetAmrState(const Commondefine::RobotState state)
{
    {
        std::lock_guard<std::mutex> lock(Task_mtx_);
        
        this->state_ = state;
    }
}

void AmrAdapter::WaitUntilWaypointOccupied()
{
    std::unique_lock<std::mutex> lock(occupy_mtx_);

    //할당된 waypoint가 없으면 바로 return 해서 데드락 문제 해결
    if(waypoints_.empty()) return;

    if (!isOccupyWaypoint_)
    {
        occupy_cv_.wait(lock, [&]()
        {
            return isOccupyWaypoint_;
        });
    }
}

bool AmrAdapter::handleWaypointArrival(const Commondefine::pose2f& pos)
{
    Commondefine::Position p = getCurrentWayPoint();
    if(p.x == -1 || p.y ==-1) return false;

    Commondefine::pose2f wp = Commondefine::convertPositionToPose(p);
    const float dx = static_cast<float>((pos.x - wp.x));
    const float dy = static_cast<float>((pos.y - wp.y));

    const float dist = std::hypot(dx, dy);

    if (dist <= _ARRIVAL_TOLERANCE_)
    {
        log_->Log(Log::LogLevel::INFO, "핑키 "+ std::to_string(robot_task_info_.robot_id) + "가 waypoint "
        + std::to_string(current_wp_idx_) + "에 도달했습니다.");

        setOccupyWayPoint(true);

        //목적지에 도착했는지 판단 유무와 다음 웨이포인트 보냄.
        sendNextpoint();

        setOccupyWayPoint(false);
    }

    return true;
}

void AmrAdapter::updatePath(const std::vector<Commondefine::Position>& new_path)
{
    {
        std::lock_guard<std::mutex> lk(waypoint_mtx_);
        
        ResetWaypoint();
        
        waypoints_ = new_path;
    }

    if(waypoints_.empty()) return;

    //여기에서 웨이포인트 간 yaw 값을 계산한다.
    const int nextiter = 1;
    for (auto it = waypoints_.begin(); it + nextiter != waypoints_.end(); ++it)
    {
        auto& cur = *it;
        auto& next = *(it + nextiter);

        cur.yaw = Commondefine::yaw(cur, next);
    }

    setOccupyWayPoint(false);
}

void AmrAdapter::setOccupyWayPoint(bool occupy)
{
    {
        std::lock_guard lock(occupy_mtx_);
        isOccupyWaypoint_ = occupy;
    }
    
    if(occupy) occupy_cv_.notify_one();

    return;
}

const bool AmrAdapter::isGoal()
{
    if(current_wp_idx_ == waypoints_.size()-1)
    {
        {
            std::lock_guard<std::mutex> lock(waypoint_mtx_);
            ResetWaypoint();
        }

        log_->Log(INFO,"AMR ID : "+ std::to_string(robot_task_info_.robot_id) +" 목적지 도착 완료");

        return true;
    }

    return false;
}

Commondefine::Position AmrAdapter::GetDestPoseToWp()
{
    return Commondefine::convertPoseToPosition(robot_task_info_.dest);
}

void AmrAdapter::ResetWaypoint()
{
    current_wp_idx_ = (0);
    waypoints_.clear(); 
}

void AmrAdapter::checkPathUpdate()
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    //항상 움직이기 전에 새로운 경로가 있는지 확인 하고 출발한다.
    bool timeout = core->waitNewPath(100ms);

    //timeout 되면 다시 checkPathUpdate를 추가해서 해당 함수가 실행 되도록 한다.
    if(!timeout)
    {
        core->assignTask(robot_task_info_.robot_id, AmrStep::check_path_update);
        return;
    }

    //lock 이 풀리면 진짜 움직이는 스텝으로 이동하게 된다.
    core->assignTask(robot_task_info_.robot_id, step_);
}

void AmrAdapter::MoveTo()
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    if(waypoints_.empty())
    {
        log_->Log(INFO,"Amr id :" + std::to_string(robot_task_info_.robot_id)+ " waypoints_ is empty");
        return;
    } 

    const int first = 0;

    //항상 움직일때 첫번째 웨이포인트 하나만 보내주고 나머지는 handleWaypointArrival 에서 다음 웨이포인트를 보내준다.
    core->publishNavGoal(robot_task_info_.robot_id, waypoints_[first]);
}

void AmrAdapter::MoveToStorage()
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    //픽업 위치에 다른 로봇이 있다면 대기 한다.
    bool timeout = core->waitCriticalSection(100ms);
    if(!timeout)
    {
        core->assignTask(robot_task_info_.robot_id, AmrStep::MoveTo_Storage);
        return;
    }

    //픽업 위치가 점유되어 있지 않다면 움직임 이동 시작
    MoveTo();
}

void AmrAdapter::MoveToChargingStation()
{
    
}



void AmrAdapter::sendNextpoint()
{
    auto core = Icore_.lock();
    if(core == nullptr) return;

    //1. 목적지에 도착한 것인가를 확인한다.
    if(isGoal())//목적지에 도착하면 경로들 모두 Reset 한다.
    {
        MoveToDone();
        return;
    }

    //2. 새로운 로봇이 할당 되어 있는지 확인한다.
    if(core->IsSyncOpen())
    {
        core->ArriveAtSyncOnce(robot_task_info_.robot_id);
        return;
    }

    //3 목적지에 도착하지도 않고 새로운 로봇이 할당되어 있지도 않으면 다음 웨이포인트를 전달한다.
    incrementWaypointIndex();

    Commondefine::Position wp = getCurrentWayPoint();
    if(wp.x == -1 || wp.y ==-1) return;

    core->publishNavGoal(robot_task_info_.robot_id, wp);

    return;
}

void AmrAdapter::MoveToDone()
{
    //여기는 각각 모든 스텝이 완료 되었을때를 의미하기 때문에 해당 스텝에서 다음 스텝을 부를때 준비 해야하는 코드들이 작성되어야 한다.
    auto core = Icore_.lock();
    if(core == nullptr) return;

    switch (step_.load())
    {
        //창고에 도착한 경우 로봇팔에게 픽업작업 요청을 하고, 상태를 실제 목적지로 변경하고 
    case Commondefine::AmrStep::MoveTo_Storage:
        core->SendPickupRequest(robot_task_info_.robot_id);    
        {
            std::lock_guard lock(current_mtx_);
            SetAmrStep(Commondefine::AmrStep::MoveTo_dst);
            current_dst = Commondefine::convertPoseToPosition(robot_task_info_.dest);
        }
        break;
        
        //
    case Commondefine::AmrStep::MoveTo_charging_station:
        {
            //충전 위치에 도달 했기 때문에 IDLE 상태로 변경하고 
            std::lock_guard lock(current_mtx_);
            SetAmrState(Commondefine::RobotState::IDLE);
            SetAmrStep(Commondefine::AmrStep::AmrStep_num);

            //로봇이 완료 됬기 때문에 일이 있는지 확인하는 역할을 한다.
            core->assignBestRobotSelector();
        }    
        break;    
    default:
        break;
    }

    return;
}

void AmrAdapter::SendPickupRequest()
{
    auto core = Icore_.lock();
    if(core == nullptr) return;

    core->assignTask(RobotArm::RobotArm2, Commondefine::RobotArmStep::buffer_to_Amr);
    return;
}

void AmrAdapter::SetBattery(float battery)
{
    std::lock_guard<std::mutex> lock(Task_mtx_);
    robot_task_info_.battery = battery;
}

float AmrAdapter::GetBattery() const
{
    std::lock_guard<std::mutex> lock(Task_mtx_);
    return robot_task_info_.battery;
}