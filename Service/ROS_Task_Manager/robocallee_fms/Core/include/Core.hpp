#pragma once
#include <future>
#include <type_traits>
#include <memory>
#include <mutex>
#include "Commondefine.hpp"
#include "ICore.hpp"
#include "Dispatcher.hpp"
#include "RosInterface.hpp"
#include "AmrAdapter.hpp"
#include "RobotArmAdapter.hpp"
#include "RequestManager.hpp"
#include "OccupancyGrid.hpp"
#include "TrafficPlanner.hpp"

namespace core
{
    class Core : public std::enable_shared_from_this<Core> , public ICore
    {
    private:
        task::Dispatcher::u_ptr                                     pdispatcher_;
        Logger::s_ptr                                               log_;
        Adapter::RobotArmAdapter::u_ptr                             pRobotArmAdapter_;
        Manager::RequestManager::u_ptr                              pRequestManager_;
        interface::RosInterface::w_ptr                              Interface_;
        std::mutex                                                  assignmtx_;
        std::mutex                                                  path_mtx_;
        std::mutex                                                  battery_mtx_;
        std::condition_variable                                     path_cv_;
        Integrated::vec<Integrated::u_ptr<Adapter::AmrAdapter>>     amr_adapters_;
        Integrated::s_ptr<traffic::TrafficPlanner>                  traffic_Planner_;
        Integrated::u_ptr<OG::OccupancyGrid>                        occupancyGrid_;
        std::atomic<bool>                                           assignNewAmr_;

    public:

        using s_ptr = std::shared_ptr<Core>;
        using u_ptr = std::unique_ptr<Core>;
        using w_ptr = std::weak_ptr<Core>;

        Core(Logger::s_ptr log , interface::RosInterface::w_ptr Interface_);
        ~Core();

        template<typename F, typename... Args>
        auto assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>;
        bool Initialize();
        bool SetAmrNextStep(int idx, Commondefine::AmrStep step) override;
        bool SetRobotArmNextStep(Commondefine::RobotArmStep step, Commondefine::shoesproperty shoes, int robot_id) override;
        bool ArmRequestMakeCall(int arm_num, int shelf_num, int robot_id , std::string action) override ;
        bool UpdateShelfInfo(Commondefine::shoesproperty incoming_shoe , int shelf_num ) override;
        int RequestCallback(const Commondefine::GUIRequest& request) override;
        bool DoneCallback(const std::string& requester, const int& customer_id) override;
        bool PoseCallback(const std::vector<Commondefine::pose2f> &pos) override;
        // bool publishNavGoal(int idx, const geometry_msgs::msg::PoseStamped wp) override;
        bool publishNavGoal(int idx, const Commondefine::Position wp) override;
        Commondefine::RobotState GetAmrState(int idx) override;
<<<<<<< Updated upstream
        float GetAmrBattery(int idx) override;
        void UpdateBattery(int idx, float precent) override;
=======
        
        float GetAmrBattery(int idx) override;

        void UpdateBattery(int idx, float precent) override;
        
>>>>>>> Stashed changes
        int GetAmrCustID(int idx);
        int GetAmrVecSize();
        void SetTaskInfo(int idx, const Commondefine::GUIRequest& request) override;
        void PlanPaths() override;
        void waitNewPath() override;
        void SetAssignNewAmr(bool assign) override { assignNewAmr_.store(assign); }
        bool GetAssignNewAmr() override { return assignNewAmr_.load(); }
        void assignWork(int amr) override;
<<<<<<< Updated upstream
        void SendTestGoal(int robot_id_1based, const Commondefine::Position wp);
        void SendTestGoal(int robot_id, int cell_x, int cell_y, float yaw_rad);
        void SendTestGoalAll(const Commondefine::Position wp);
        bool IsReachedNow(int robot_id, float pos_tol_m=0.15f, float yaw_tol_rad=0.26f); // ~15도
=======

        void SendTestGoal(int robot_id_1based, const Commondefine::Position wp);

        void SendTestGoal(int robot_id, int cell_x, int cell_y, float yaw_rad);

        void SendTestGoalAll(const Commondefine::Position wp);
        
        bool IsReachedNow(int robot_id, float pos_tol_m=0.15f, float yaw_tol_rad=0.26f); // ~15도

>>>>>>> Stashed changes
    };
    
    // Template implementation
    template<typename F, typename... Args>
    auto Core::assignTask(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type>
    {
        std::lock_guard<std::mutex> lock(assignmtx_);
        using return_type = typename std::invoke_result<F, Args...>::type;
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...) );
        std::future<return_type> fut = task->get_future();
        pdispatcher_->Enqueue([task]() { (*task)(); });
        return fut;
    }
};