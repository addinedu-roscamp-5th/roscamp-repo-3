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

        Commondefine::RobotTaskInfo               task_info_;
        

    public:
        using u_ptr = Integrated::u_ptr<AmrAdapter>;

        
        AmrAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log, const std::string& name);
        ~AmrAdapter();

        void UpdateTaskInfo(const Commondefine::RequestInfo& request);

        Commondefine::RobotTaskInfo& GetTaskInfo();

    };
};