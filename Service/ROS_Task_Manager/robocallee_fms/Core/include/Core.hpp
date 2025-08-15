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
#include "StorageManager.hpp"
#include "PathSyncManager.hpp"
#include "TrafficPlanner.hpp"

namespace core
{
    class Core : public std::enable_shared_from_this<Core> , public ICore
    {
    private:
        task::Dispatcher::u_ptr                                         pdispatcher_;
        Logger::s_ptr                                                   log_;
        
        Manager::RequestManager::u_ptr                                  pRequestManager_;
        Manager::StorageManager::u_ptr                                  pStorageManager_;
        Manager::PathSyncManager::u_ptr                                 pPathSyncManager_;
        
        
        interface::RosInterface::w_ptr                                  Interface_;

        Integrated::vec<Integrated::u_ptr<Adapter::RobotArmAdapter>>    RobotArm_Adapters_;

        std::mutex                                                      assignmtx_;
        std::mutex                                                      path_mtx_;
        std::condition_variable                                         path_cv_;
        
        
        Integrated::vec<Integrated::u_ptr<Adapter::AmrAdapter>>         amr_adapters_;
        Integrated::u_ptr<traffic::TrafficPlanner>                      traffic_Planner_;
    public:
        using s_ptr = std::shared_ptr<Core>;
        using u_ptr = std::unique_ptr<Core>;
        using w_ptr = std::weak_ptr<Core>;

        Core(Logger::s_ptr log , interface::RosInterface::w_ptr Interface_);
        ~Core();

        bool Initialize();

        template<typename F, typename... Args>
        auto assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>;

        bool assignTask(int idx, Commondefine::AmrStep step) override;

        bool assignTask(int idx, Commondefine::RobotArmStep step) override;

        bool assignPath(Integrated::Task task) override;

        void assignWork(int amr, Commondefine::GUIRequest r) override;

        //----------------Call Back function

        bool ArmRequestMakeCall(Commondefine::RobotArm arm, int shelf_num, int robot_id, std::string action) override ;

        int RequestCallback(const Commondefine::GUIRequest& request) override;
        
        bool DoneCallback(const std::string& requester, const int& customer_id) override;

        bool PoseCallback(const std::vector<Commondefine::pose2f> &pos) override;

        bool ArmDoneCallback(Commondefine::ArmRequest request) override;

        //----------------Call Back function

        bool publishNavGoal(int idx, const Commondefine::Position wp) override;

        Commondefine::RobotState GetAmrState(int idx) override;
        
        void UpdateBattery(int idx, float percent) override;

        int GetAmrBattery(int idx) override;
        
        int GetAmrCustID(int idx);

        int GetAmrVecSize();

        void SetTaskInfo(int idx, const Commondefine::GUIRequest& request) override;

        void PlanPaths();

        void assignPlanPaths() override;

        bool waitNewPath(std::chrono::milliseconds ms) override;

        void assignBestRobotSelector() override;

        bool SendPickupRequest(int idx) override;
        
        //----------------Storage--------------------------

        bool setStorageRequest(Commondefine::StorageRequest& Request) override;
        
        bool findStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request) override;

        bool setStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest Request) override;

        bool getStorage(Commondefine::ContainerType Container , Commondefine::StorageRequest& Request) override;

        int findEmptyStorage(Commondefine::ContainerType Container) override;

        bool waitCriticalSection(std::chrono::milliseconds ms) override;

        bool waitWorkOnlyOnce(std::chrono::milliseconds ms) override;

        void setWorkOnlyOnce(bool flag) override;

        //--------------------------PathSyncManager--------------------------

        int CurrentActiveRobotCount() override;

        void ReplanAndBroadcast() override;

        bool IsSyncOpen() override;
        
        void ArriveAtSyncOnce(int robot_id) override;

        void OpenSyncWindow() override;
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


