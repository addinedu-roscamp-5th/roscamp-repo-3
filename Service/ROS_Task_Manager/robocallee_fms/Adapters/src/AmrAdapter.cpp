#include "AmrAdapter.hpp"

using namespace Adapter;
using namespace Commondefine;
using namespace Integrated;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const int id)
    :Icore_(Icore), log_(log), isOccupyWaypoint_(false)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");

    robot_task_info_.robot_id = id;
    
    current_wp_idx_.store(0);
}

AmrAdapter::~AmrAdapter()
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 소멸");
}


Commondefine::RobotTaskInfo& AmrAdapter::GetTaskInfo()
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

    if (!isOccupyWaypoint_.load())
    {
        occupy_cv_.wait(lock, [&]()
        {
            return isOccupyWaypoint_.load();
        });
    }
}

bool AmrAdapter::handleWaypointArrival(const Commondefine::pose2f& pos)
{
    auto core = Icore_.lock();
    if(core == nullptr) return false;

    Commondefine::Position p = getCurrentWayPoint();
    if(p.x == -1 || p.y ==-1) return false;

    Commondefine::pose2f wp = Commondefine::convertPositionToPose(p);
    const float dx = static_cast<float>((pos.x - wp.x));
    const float dy = static_cast<float>((pos.y - wp.y));

    const float dist = std::hypot(dx, dy);

    if (dist <= _ARRIVAL_TOLERANCE_)
    {
        log_->Log(Log::LogLevel::INFO, "핑키 "+ std::to_string(robot_task_info_.robot_id) + "가 waypoint "
        + std::to_string(current_wp_idx_.load()) + "에 도달했습니다.");

        setOccupyWayPoint(true);

        //목적지에 도착했는지 판단 유무와 다음 웨이포인트 보냄.
        sendNextpoint();

        setOccupyWayPoint(false);
    }
}

void AmrAdapter::updatePath(const std::vector<Commondefine::Position>& new_path)
{
    {
        std::lock_guard<std::mutex> lk(waypoint_mtx_);
        
        ResetWaypoint();
        
        waypoints_ = new_path;
    }

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
    isOccupyWaypoint_.store(occupy);

    if(occupy) occupy_cv_.notify_one();
}

const bool AmrAdapter::isGoal()
{
    if(current_wp_idx_.load() == waypoints_.size())
    {
        {
            std::lock_guard<std::mutex> lock(waypoint_mtx_);
            ResetWaypoint();
        }

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
    current_wp_idx_.store(0);
    waypoints_.clear(); 
}

//첫번째 waypoint로 움직인다.
void AmrAdapter::MoveTo()
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    if(waypoints_.empty()) return;

    const int first = 0;
    core->publishNavGoal(robot_task_info_.robot_id, waypoints_[first]);
}

void AmrAdapter::MoveToStorage()
{
    
    auto core = Icore_.lock();
    if (core == nullptr) return;

    //픽업 위치에 다른 로봇이 있다면 대기 한다.
    core->waitCriticalSection();

    //항상 움직일때 첫번째 웨이포인트 하나만 보내주고 나머지는 handleWaypointArrival 에서 다음 웨이포인트를 보내준다.
    MoveTo();
}

void AmrAdapter::MoveToChargingStation()
{

}



void AmrAdapter::sendNextpoint()
{
    auto core = Icore_.lock();
    if(core == nullptr) return;

    if(isGoal())
    {
        MoveToDone();
        return;
    } 

    core->waitNewPath();

    incrementWaypointIndex();

    Commondefine::Position wp = getCurrentWayPoint();
    if(wp.x == -1 || wp.y ==-1) return;

    core->publishNavGoal(robot_task_info_.robot_id, wp);
}

void AmrAdapter::MoveToDone()
{
    switch (step_.load())
    {
    case Commondefine::AmrStep::MoveTo_Storage:
        SendPickupRequest();

        break;
    case Commondefine::AmrStep::MoveTo_charging_station:
        //로봇의 상태를 리턴 형태로 변환 해준다.
        state_.store(Commondefine::RobotState::RETURN);
        break;
    case Commondefine::AmrStep::MoveTo_dst:
        break;
    
    default:
        break;
    }
}

void AmrAdapter::SendPickupRequest()
{
    auto core = Icore_.lock();
    if(core == nullptr) return;

    core->assignTask(Commondefine::RobotArmStep::buffer_to_Amr);
    return;
}