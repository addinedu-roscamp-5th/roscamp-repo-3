#include "AmrAdapter.hpp"

using namespace Adapter;
using namespace Commondefine;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const int id)
    :Icore_(Icore), log_(log), isOccupyWaypoint_(false)
{
    log_->Log(Log::LogLevel::INFO,"AmrAdapter 객체 생성");

    robot_task_info_.robot_id = id;
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

        robot_task_info_.robot_state = Commondefine::RobotState::BUSY;
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

void AmrAdapter::SetAmrState(const Commondefine::RobotState& state)
{
    {
        std::lock_guard<std::mutex> lock(Task_mtx_);
        
        robot_task_info_.robot_state = state;
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
    if(core == nullptr) 
    {
        return false;
    }

    Commondefine::Position p = getCurrentWayPoint();
    if(p.x == -1 || p.y ==-1) return false;

    Commondefine::pose2f wp = convertWaypoint(p);
    float dx = float(pos.x - static_cast<float>(p.x));
    float dy = float(pos.y - static_cast<float>(p.y));
    const float dist = std::hypot(dx, dy);
    
    if (dist <= _ARRIVAL_TOLERANCE_)
    {
        log_->Log(Log::LogLevel::INFO, "핑키 "+ std::to_string(robot_task_info_.robot_id) + "가 waypoint "
        + std::to_string(current_wp_idx_.load()) + "에 도달했습니다.");

        setOccupyWayPoint(true);

        if (isGoal()){ return true; }

        core->waitNewPath();

        incrementWaypointIndex();

         // ★ Core::publishNavGoal 인덱스 주의 (0-based 기대)
        // int core_idx = robot_task_info_.robot_id - 1; // robot_id가 1-based라면
        // core->publishNavGoal(core_idx, getCurrentWayPoint());

        core->publishNavGoal(robot_task_info_.robot_id, getCurrentWayPoint());

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

    const int nextiter = 1;
    for (auto it = waypoints_.begin(); it + nextiter != waypoints_.end(); ++it)
    {
        auto& cur = *it;
        auto& next = *(it + nextiter);

        cur.yaw = Commondefine::yaw(cur, next);
    }

    setOccupyWayPoint(false);

    if (auto core = Icore_.lock())
    {
       // core->SetAmrNextStep(robot_task_info_.robot_id, Commondefine::AmrStep::MoveTo_dest1);
    }

}

void AmrAdapter::MoveTo_dest1(int robot_id)
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest1() 호출");

    // core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest1);
    core->publishNavGoal(robot_task_info_.robot_id, { (int)robot_task_info_.dest.x , (int)robot_task_info_.dest.y } );


    
    log_->Log(Log::LogLevel::INFO, "sent dest1 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        // std::to_string(robot_task_info_.dest1.y) + ") to pinky " +
        std::to_string( (int)robot_task_info_.dest.x ) + std::to_string(  (int)robot_task_info_.dest.y ) + ") to pinky " +

        std::to_string(robot_task_info_.robot_id)
    );

    if (robot_task_info_.requester == "customer")
    {   
        core->SetRobotArmNextStep(Commondefine::RobotArmStep::buffer_to_pinky, dummy_shoe, robot_task_info_.robot_id);
        return;
    }
}

void AmrAdapter::MoveTo_dest2(int robot_id)
{
    auto core = Icore_.lock();
    if (core == nullptr) return;

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest2() 호출");

    // core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest2);
    core->publishNavGoal(robot_task_info_.robot_id, { (int)robot_task_info_.dest.x , (int)robot_task_info_.dest.y } );


    log_->Log(Log::LogLevel::INFO, "sent dest2 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        // std::to_string(robot_task_info_.dest2.y) + ") to pinky " +
        std::to_string( (int)robot_task_info_.dest.x ) + std::to_string(  (int)robot_task_info_.dest.y ) + ") to pinky " +
        std::to_string(robot_task_info_.robot_id)
    );

    if (robot_task_info_.requester == "employee")
    {
        core->SetRobotArmNextStep(Commondefine::RobotArmStep::pinky_to_buffer, dummy_shoe, robot_task_info_.robot_id);
        return;
    }
}


void AmrAdapter::MoveTo_dest3(int robot_id)
{
    auto core = Icore_.lock();
    if(core == nullptr) return;

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest3() 호출");

    // core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest3  );
    // core->publishNavGoal(robot_task_info_.robot_id, {robot_task_info_.dest3.x , robot_task_info_.dest3.y}  );
    core->publishNavGoal(robot_task_info_.robot_id, { (int)robot_task_info_.dest.x , (int)robot_task_info_.dest.y } );


    log_->Log(Log::LogLevel::INFO, "sent dest3 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        // std::to_string(robot_task_info_.dest3.y) + ") to pinky " +
        std::to_string( (int)robot_task_info_.dest.x ) + std::to_string(  (int)robot_task_info_.dest.y ) + ") to pinky " +
        std::to_string(robot_task_info_.robot_id)
    );
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
            std::lock_guard<std::mutex> lk(waypoint_mtx_);
            ResetWaypoint();
        }

        return true;
    }

    return false;
}

void AmrAdapter::SetCurrentPosition(Commondefine::pose2f p)
{
    {
        std::lock_guard lock(current_position_mtx_);
        robot_task_info_.current_position = std::move(p);
    }

    return;
}

Commondefine::Position AmrAdapter::GetCurrentPosition()
{
    Commondefine::Position p;

    p.x = (robot_task_info_.current_position.x - (_MAP_RESOLUTION_ / 2.0)) / _MAP_RESOLUTION_ + 1;
    p.y = (robot_task_info_.current_position.y - (_MAP_RESOLUTION_ / 2.0)) / _MAP_RESOLUTION_ + 1;

    return p;
}

Commondefine::Position AmrAdapter::GetDestPosition()
{
    Commondefine::Position pose;

    pose.x = static_cast<int>(robot_task_info_.dest.x);
    pose.y = static_cast<int>(robot_task_info_.dest.y);
    return pose;
}

void AmrAdapter::ResetWaypoint()
{
    current_wp_idx_.store(0);

    waypoints_.clear(); 
}

Commondefine::pose2f AmrAdapter::convertWaypoint(Commondefine::Position wp)
{
    pose2f p2f;

    p2f.x = (wp.x - 1) * _MAP_RESOLUTION_ + _MAP_RESOLUTION_ / 2.0;
    p2f.y = (wp.y - 1) * _MAP_RESOLUTION_ + _MAP_RESOLUTION_ / 2.0;
    
    return p2f;
}