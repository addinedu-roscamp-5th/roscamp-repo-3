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
        

    public:
        using u_ptr = Integrated::u_ptr<AmrAdapter>;

        
        AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const std::string& name);
        ~AmrAdapter();

        void SetTaskInfo(const Commondefine::GUIRequest& request);

        Commondefine::RobotTaskInfo& GetTaskInfo();

        void SetAmrState(const Commondefine::RobotState& state);

    };
};