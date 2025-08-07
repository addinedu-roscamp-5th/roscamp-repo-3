#include "AmrAdapter.hpp"

using namespace Adapter;

AmrAdapter::AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const int& id)
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
        std::lock_guard<std::mutex> lock(mtx_);

        robot_task_info_.robot_state = Commondefine::RobotState::BUSY;
        robot_task_info_.shoes_property = request.shoes_property;
    
        robot_task_info_.requester = request.requester;
        robot_task_info_.customer_id = request.customer_id;
    }
    // std::ostringstream oss;
    // oss << ",color=" << robot_task_info_.shoes_property.color
    //     << ",model=" << robot_task_info_.shoes_property.model
    //     << ",size=" << robot_task_info_.shoes_property.size
    //     << ",robot_id=" << robot_task_info_.robot_id;

    std::string msg = "color = " + robot_task_info_.shoes_property.color +
    ", model = " + robot_task_info_.shoes_property.model + "size = " + std::to_string(robot_task_info_.shoes_property.size) +
    ", robot_id = " + std::to_string(robot_task_info_.robot_id);

    log_->Log(Log::LogLevel::INFO, msg);
    log_->Log(Log::LogLevel::INFO,"SetTaskInfo 완료");
}

void AmrAdapter::SetAmrState(const Commondefine::RobotState& state)
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        
        robot_task_info_.robot_state = state;
    }
}

bool AmrAdapter::handleWaypointArrival(const Commondefine::pose2f& pos)
{
    auto core = Icore_.lock();
    if(core == nullptr) return false;

    Commondefine::Position p = getCurrentWayPoint();
    if(p.x == -1 || p.y ==-1) return false;

    float dx = float(pos.x - static_cast<float>(p.x));
    float dy = float(pos.y - static_cast<float>(p.y));

    float dist = std::hypot(dx, dy);

    if (dist <= _ARRIVAL_TOLERANCE_)
    {
        log_->Log(Log::LogLevel::INFO, "핑키 "+ std::to_string(robot_task_info_.robot_id) + "가 waypoint " 
        + std::to_string(current_wp_idx_.load()) + "에 도달했습니다.");

        setOccupyWayPoint(true);
        
        if (isGetGoal()) { return true;}

        //무조건 increment 하기 전에 불려져야함 !!
        yaw();

        incrementWaypointIndex();

        core->publishNavGoal(robot_task_info_.robot_id, getCurrentWayPoint());

        setOccupyWayPoint(false);
    }
}


void AmrAdapter::WaitUntilWaypointOccupied()
{
    std::unique_lock<std::mutex> lock(occupy_mtx_);

    if (!isOccupyWaypoint_.load())
    {
        occupy_cv_.wait(lock, [&]()
        {
            return isOccupyWaypoint_.load();
        });
    }
}

void AmrAdapter::updatePath(const std::vector<Commondefine::Position>& new_path)
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        waypoints_ = new_path;
        ResetCurrentWpIndex();
    }

    setOccupyWayPoint(false);

    if (auto core = Icore_.lock())
    {
        core->SetAmrNextStep(robot_task_info_.robot_id, Commondefine::AmrStep::MoveTo_dest1);
    }

}

void AmrAdapter::MoveTo_dest1(int robot_id)
{
    auto core = Icore_.lock();
    if (!core) return;
    core->waitNewPath();

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest1() 호출");

    core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest1);

    
    log_->Log(Log::LogLevel::INFO, "sent dest1 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        std::to_string(robot_task_info_.dest1.y) + ") to pinky " +
        std::to_string(robot_task_info_.robot_id)
    );

    if (robot_task_info_.requester == "customer")
    {
            core->SetRobotArmNextStep(Commondefine::RobotArmStep::buffer_to_pinky, robot_task_info_.robot_id) ;

            return;
        }
}

void AmrAdapter::MoveTo_dest2(int robot_id)
{
    auto core = Icore_.lock();
    if (!core) return;

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest2() 호출");

    core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest2);

    log_->Log(Log::LogLevel::INFO, "sent dest2 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        std::to_string(robot_task_info_.dest2.y) + ") to pinky " +
        std::to_string(robot_task_info_.robot_id)
    );

    if (robot_task_info_.requester == "employee")
    {
        core->SetRobotArmNextStep(Commondefine::RobotArmStep::pinky_to_buffer, robot_task_info_.robot_id);
        return;
    }
}


void AmrAdapter::MoveTo_dest3(int robot_id)
{
    auto core = Icore_.lock();
    if(!core) return;

    log_->Log(Log::LogLevel::INFO, "MoveTo_dest3() 호출");

    core->publishNavGoal(robot_task_info_.robot_id, robot_task_info_.dest3);

    log_->Log(Log::LogLevel::INFO, "sent dest3 (" +
        std::to_string(robot_task_info_.robot_id) + ", " +
        std::to_string(robot_task_info_.dest3.y) + ") to pinky " +
        std::to_string(robot_task_info_.robot_id)
    );
}

bool AmrAdapter::yaw()
{
    auto way = getWaypoints();
    int index = current_wp_idx_.load();

    int size = current_wp_idx_.size() - 1;
    if(index > size) return false;

    const Commondefine::Position cur = way[index];
    const Commondefine::Position& next = way[index + 1];

    next.yaw = Commondefine::yaw(cur, next);

    return true;
}