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

        std::mutex                                                  assignmtx_;
        interface::RosInterface::w_ptr                              Interface_;
        
        Integrated::vec<Integrated::u_ptr<Adapter::AmrAdapter>>     amr_adapters_;
        Integrated::s_ptr<traffic::TrafficPlanner>                  traffic_Planner_;
        Integrated::u_ptr<OG::OccupancyGrid>                        occupancyGrid_;


    public:
        using s_ptr = std::shared_ptr<Core>;
        using u_ptr = std::unique_ptr<Core>;
        using w_ptr = std::weak_ptr<Core>;

        Core(Logger::s_ptr log , interface::RosInterface::s_ptr Interface_);
        ~Core();

        template<typename F, typename... Args>
        auto assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>;

        bool Initialize();

        bool SetAmrNextStep(int idx, Commondefine::AmrStep step) override;

        
        bool SetRobotArmNextStep(Commondefine::RobotArmStep step , Commondefine::shoesproperty shoe_info , int pinky_num ) override;
        // bool SetRobotArmNextStep(Commondefine::RobotArmStep step) override;
        
        bool ArmRequestMakeCall(int arm_num, int shelf_num, int pinky_num , std::string action) override ;

        int RequestCallback(const Commondefine::GUIRequest& request) override;
        
        bool DoneCallback(const std::string& requester, const int& customer_id) override;

        bool PoseCallback(const Commondefine::pose2f &pos, int pinky_id) override;

        Commondefine::RobotState GetAmrState(int idx) override;
        
        int GetAmrBattery(int idx) override;
        
        int GetAmrCustID(int idx);

        int GetAmrVecSize();

        void SetTaskInfo(int idx, const Commondefine::GUIRequest& request) override;

        void handleWaypointArrival(int pinky_id);

        // void SetAmrState(int idx) override;
    
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


