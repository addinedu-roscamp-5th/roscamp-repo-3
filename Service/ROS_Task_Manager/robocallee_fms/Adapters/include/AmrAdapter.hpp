#pragma once

#include "Integrated.hpp"
#include "Commondefine.hpp"
#include "ICore.hpp"

#include <mutex>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <string>
#include <cmath>


namespace Adapter
{
    class AmrAdapter
    {
    private:
        Integrated::w_ptr<core::ICore>       Icore_;
        Logger::s_ptr                        log_;
        Commondefine::RobotTaskInfo          robot_task_info_;
        std::mutex                           mtx_;

        // Planner → Adapter 로 받은 웨이포인트
        std::vector<Commondefine::Position>  waypoints_;
        std::atomic<int>                     current_wp_idx_{0};
        std::atomic<Commondefine::Position>  current_goal_;

        // 웨이포인트 점유 플래그
        std::atomic<bool>                    isOccupyWaypoint_;
        std::mutex                           occupy_mtx_;
        std::condition_variable              occupy_cv_;


    public:
        using u_ptr = Integrated::u_ptr<AmrAdapter>;

        AmrAdapter(Integrated::w_ptr<core::ICore> Icore,
                Logger::s_ptr log,
                const int& id);
        ~AmrAdapter();

        // Request → AMR task 정보 설정
        void SetTaskInfo(const Commondefine::GUIRequest& request);
        Commondefine::RobotTaskInfo& GetTaskInfo();

        // AMR 상태 변경
        void SetAmrState(const Commondefine::RobotState& state);

        // Waypoint 로직
        bool handleWaypointArrival(const Commondefine::pose2f& pos);

        void WaitUntilWaypointOccupied();

        void updatePath(const std::vector<Commondefine::Position>& new_path);

        void ResetCurrentWpIndex(){current_wp_idx_.store(0);}

        int  GetCurrentWpIndex() const;  

        void setOccupyWayPoint(bool occupy)
        {
            isOccupyWaypoint_.store(occupy);

            if(occupy) occupy_cv_.notify_one();
        }

        bool yaw();

        // 경로 조회
        const std::vector<Commondefine::Position>& getWaypoints() const;
        Commondefine::Position                     getCurrentWayPoint() { return waypoints_[current_wp_idx_];}
        Commondefine::Position                     getCurrentGoal(){ return *(waypoints_.end());}
        bool                                       isGetGoal() const;
        void incrementWaypointIndex()              { ++current_wp_idx_; }

        // 단계별 이동
        void MoveTo_dest1(int robot_id);
        void MoveTo_dest2(int robot_id);
        void MoveTo_dest3(int robot_id);
    };
} // namespace Adapter
