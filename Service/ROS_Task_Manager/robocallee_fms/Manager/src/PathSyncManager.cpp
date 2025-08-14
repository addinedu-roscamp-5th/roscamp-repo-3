#include "PathSyncManager.hpp"

using namespace Manager;
using namespace Commondefine;
using namespace Integrated;

//#define _USE_ASSIGN_TASK_

PathSyncManager::PathSyncManager(Integrated::w_ptr<core::ICore> icore,int max_robot, Logger::s_ptr log)
    : icore_(icore), arrived_in_window_(max_robot + 1, false), log_(log)
{
    
}

PathSyncManager::~PathSyncManager(){}

void PathSyncManager::OpenSyncWindow()
{
    open_impl();
}

void PathSyncManager::ArriveAtSyncOnce(int robot_id)
{
#ifdef _USE_ASSIGN_TASK_
    auto core = icore_.lock();
    if(core ==nullptr) return;

    core->assignPath([this, robot_id]{ arrive_impl(robot_id); });
#else
    arrive_impl(robot_id); 
#endif
    
}

void PathSyncManager::ShrinkExpectedAsync(int new_expected)
{
#ifdef _USE_ASSIGN_TASK_
    auto core = icore_.lock();
    if(core ==nullptr) return;

    core->assignPath([this, new_expected]{ shrink_impl(new_expected); });
#else
    shrink_impl(new_expected);
#endif
}

bool PathSyncManager::IsSyncOpen() 
{
    std::lock_guard<std::mutex> lock(mtx_);
    return requestNewPath_;
}

void PathSyncManager::open_impl()
{
    auto core = icore_.lock();
    if (!core) return;

    const int expected_now = core->CurrentActiveRobotCount();
    bool trigger = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        requestNewPath_ = true;
        expected_  = expected_now;
        remaining_ = expected_now;

        // ★ 오픈 직후 참가자가 0이면 즉시 트리거하도록 플래그
        if (remaining_ == 0)
            trigger = true;
    }

    if (trigger)
    {
        // ★ 반드시 락 밖에서 replan 호출 스케줄
        core->assignPath([this] { replan_and_close_impl(); });
    }
}

void PathSyncManager::arrive_impl(int robot_id)
{
    bool trigger = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!requestNewPath_) return;

        if (robot_id < 0 || robot_id >= (int)arrived_in_window_.size()) return;

        if (arrived_in_window_[robot_id]) return;           // 이미 이번 윈도우에서 집계됨(중복 방지)

        arrived_in_window_[robot_id] = true;                // 이번 윈도우의 "첫 도착" 표식

        if (remaining_ > 0 && --remaining_ == 0)
        {
            trigger = true;                                 // 마지막 도착자 → 리플랜 트리거
        }
    }

    if (trigger)
    {
        auto core = icore_.lock();
        if(core ==nullptr) return;

        core->assignPath([this]{ replan_and_close_impl(); });
    }
}

void PathSyncManager::replan_and_close_impl()
{
    auto core = icore_.lock();
    if(core ==nullptr) return;
    
    // 1) 실제 경로 재계획 + 브로드캐스트 (락 없이)
    core->ReplanAndBroadcast();

    // 2) 윈도우 닫기 (라운드 종료)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        requestNewPath_ = false;

        std::fill(arrived_in_window_.begin(), arrived_in_window_.end(), false);
    }
}

void PathSyncManager::shrink_impl(int new_expected)
{
    bool trigger = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        int arrived = expected_ - remaining_;
        expected_ = new_expected;

        int need = new_expected - arrived;
        if (need < 0) need = 0;

        int before = remaining_;
        remaining_ = need;

        if (requestNewPath_ && before != 0 && remaining_ == 0)
        {
            trigger = true;
        }
    }

    if (trigger)
    {
        auto core = icore_.lock();
        if(core ==nullptr) return;

        core->assignPath([this]{ replan_and_close_impl(); });
    }
}

bool PathSyncManager::checkSyncWindow()
{
    return true;
}