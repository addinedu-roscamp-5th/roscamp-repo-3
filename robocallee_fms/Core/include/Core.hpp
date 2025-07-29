#pragma once

#include <future>
#include <type_traits>

#include "ICore.hpp"
#include "Dispatcher.hpp"
#include "Commondefine.hpp"
#include "AmrAdapter.hpp"
#include "RobotArmAdapter.hpp"
#include "RosInterface.hpp"
#include "RequestManager.hpp"

using namespace Integrated;

namespace core
{
    class Core : public std::enable_shared_from_this<Core> , public ICore
    {
    private:
<<<<<<< Updated upstream:robocallee_fms/Core/include/Core.hpp
        task::Dispatcher::u_ptr             pdispatcher_;
        Logger::s_ptr                       log_;
        Adapter::AmrAdapter::u_ptr          pAmrAdapter_;
        Adapter::RobotArmAdapter::u_ptr     pRobotArmAdapter_;
        Manager::RequestManager::u_ptr      pRequestManager_;     ///////////////////////////
        std::mutex                          assignmtx_;
        interface::RosInterface::w_ptr      Interface_;
=======
        task::Dispatcher::u_ptr                                 pdispatcher_;
        Logger::s_ptr                                           log_;
        std::array<Adapter::AmrAdapter::u_ptr, _MAX_AMR_NUM_>   pAmrAdapters_;
        Adapter::RobotArmAdapter::u_ptr                         pRobotArmAdapter_;
        Manager::RequestManager::u_ptr                          pRequestManager_;
        std::mutex                                              assignmtx_;
        interface::RosInterface::w_ptr                          Interface_;
>>>>>>> Stashed changes:Robocallee_Fms/Core/include/Core.hpp
    
    public:
        using s_ptr = std::shared_ptr<Core>;
        using u_ptr = std::unique_ptr<Core>;
        using w_ptr = std::weak_ptr<Core>;
<<<<<<< Updated upstream:robocallee_fms/Core/include/Core.hpp
        using ReqServiceType = robocallee_fms::srv::ShoeRequest;    ///////////////
        using DoneServiceType = robocallee_fms::srv::DoneMsg;
=======
        using ReqServiceType = robocallee_fms::srv::ProcessRequest;
        using DoneServiceType = robocallee_fms::srv::ProcessDone;
>>>>>>> Stashed changes:Robocallee_Fms/Core/include/Core.hpp
    

        Core(Logger::s_ptr log , interface::RosInterface::s_ptr Interface_);
        ~Core();

        template<typename F, typename... Args>
        auto assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>;

        bool Initialize();

        bool SetAmrNextStep(Commondefine::AmrStep step) override;

        bool SetRobotArmNextStep(Commondefine::RobotArmStep step) override;
        
<<<<<<< Updated upstream:robocallee_fms/Core/include/Core.hpp
        ////////
        bool RequestCallback(const Commondefine::GUIRequest& request) override;
=======
        void HandleRequestService(const std::shared_ptr<ReqServiceType::Request>& request,
                                  std::shared_ptr<ReqServiceType::Response>& response);
>>>>>>> Stashed changes:Robocallee_Fms/Core/include/Core.hpp
        
        bool DoneCallback(const std::string& requester) override;

        
    };

    template<typename F, typename... Args>
    auto Core::assignTask(F&& f, Args&&... args)-> std::future<typename std::invoke_result<F, Args...>::type>
    {
        std::lock_guard lock(assignmtx_);

        using return_type = typename std::invoke_result<F, Args...>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>
        (
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> fut = task->get_future();

        pdispatcher_->Enqueue([task]() { (*task)();} );
        
        return fut;
    }
};