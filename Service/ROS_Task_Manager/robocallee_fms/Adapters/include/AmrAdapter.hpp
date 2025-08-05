#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "ICore.hpp"

namespace Adapter
{
    class AmrAdapter
    {
    private:
        Integrated::w_ptr<core::ICore>            Icore_;
        Logger::s_ptr                             log_;

        Commondefine::RobotTaskInfo               robot_task_info_;
        std::mutex                                mtx_;

        std::vector<Commondefine::pose2f>         waypoints_;
        int                                       current_wp_idx_{0};

    public:
        using u_ptr = Integrated::u_ptr<AmrAdapter>;

        
        AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const std::string& name);
        ~AmrAdapter();

        void SetTaskInfo(const Commondefine::GUIRequest& request);

        Commondefine::RobotTaskInfo& GetTaskInfo();

        void SetAmrState(const Commondefine::RobotState& state);
        void onWaypointReached(int pinky_id);

        const std::vector<Commondefine::pose2f>& getWaypoints() const { return waypoints_; }
        int GetCurrentWaypointIndex() const                           { return current_wp_idx_; }
        void incrementWaypointIndex()                                 { ++current_wp_idx_; }

        void planpath(const Commondefine::pose2f& start, const Commondefine::pose2f& goal);

        void MoveTo_dest1();
        void MoveTo_dest2();
        void MoveTo_dest3();
    };
};