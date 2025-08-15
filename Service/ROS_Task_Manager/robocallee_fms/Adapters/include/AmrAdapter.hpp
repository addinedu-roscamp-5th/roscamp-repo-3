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
        Integrated::w_ptr<core::ICore>        Icore_;
        Logger::s_ptr                         log_;
        Commondefine::RobotTaskInfo           robot_task_info_;
        mutable std::mutex                    Task_mtx_;

        // Planner → Adapter 로 받은 웨이포인트
        std::mutex                            waypoint_mtx_;
        std::vector<Commondefine::Position>   waypoints_;
        int                                   current_wp_idx_;

        std::mutex                            current_mtx_;
        Commondefine::Position                current_waypoint_;
        Commondefine::pose2f                  current_pose_;
        Commondefine::Position                current_dst_;

        std::atomic<Commondefine::AmrStep>    step_;
        std::atomic<Commondefine::RobotState> state_;

    public:
        using u_ptr = Integrated::u_ptr<AmrAdapter>;

        AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const int id);
        ~AmrAdapter();

        // Request → AMR task 정보 설정
        void SetTaskInfo(const Commondefine::GUIRequest& request);
        Commondefine::RobotTaskInfo& GetTaskInfo();

        const Commondefine::RobotTaskInfo& GetTaskInfo() const;

        // step
        void SetAmrStep(const Commondefine::AmrStep step){step_.store(step);}
        const Commondefine::AmrStep GetAmrStep(){return step_.load();}

        // AMR
        void SetAmrState(const Commondefine::RobotState state);
        const Commondefine::RobotState GetAmrState(){return state_.load();}

        void SetBattery(float battery);
    
        float GetBattery() const;
    

        // Waypoint 로직
        bool handleWaypointArrival(const Commondefine::pose2f& pos);

        void updatePath(const std::vector<Commondefine::Position>& new_path);

        void ResetWaypoint();

        int  GetCurrentWpIndex() const;  

        void setOccupyWayPoint(bool occupy);

        // 경로 조회
        const std::vector<Commondefine::Position>& getWaypoints() const;
        
        Commondefine::Position getCurrentWayPoint() 
        { 
            std::lock_guard lock(current_mtx_);

            if(waypoints_.empty() || current_wp_idx_ > waypoints_.size())
            {
                Commondefine::Position p = {-1,-1,-1};
                return p;
            }

            return waypoints_[current_wp_idx_];
        }
        Commondefine::Position getEndWayPoint(){ return *(waypoints_.end());}

        const bool isGoal();
        
        void incrementWaypointIndex() 
        {
            std::lock_guard lock(current_mtx_);
            ++current_wp_idx_;
        }

        void SetCurrentPose(Commondefine::pose2f p)
        {
            std::lock_guard lock(current_mtx_);
            current_pose_ = p;
        }
        Commondefine::pose2f GetCurrentPose(){return current_pose_;}
        Commondefine::Position GetCurrentPoseToWp()
        {
            std::lock_guard lock(current_mtx_);
            return Commondefine::convertPoseToPosition(current_pose_);
        }

        Commondefine::Position GetDestPoseToWp();
        Commondefine::pose2f GetDestPose(){return robot_task_info_.dest;}

        void SetCurrentDst(Commondefine::Position p){ std::lock_guard lock(current_mtx_); current_dst_ = p;}
        
        Commondefine::Position GetCurrentDst(){return current_dst_;}

        void checkPathUpdate();

        void MoveTo();

        void MoveToStorage();

        void MoveToChargingStation();

        void sendNextpoint();

        void SendPickupRequest();

        void MoveToDone();
    };

}
