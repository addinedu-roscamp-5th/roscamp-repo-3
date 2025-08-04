#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "ICore.hpp"

namespace Manager
{
    class RequestManager
    {
    private:
        Integrated::w_ptr<core::ICore>              Icore_;
        Logger::s_ptr                               log_;
        std::queue<Commondefine::GUIRequest>        request_queue_;
        std::mutex                                  mtx_;

    public:
        using u_ptr = Integrated::u_ptr<RequestManager>;

        RequestManager(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log);
        ~RequestManager();

        int EnqueueRequest(const Commondefine::GUIRequest& r);

        void PopRequest(Commondefine::GUIRequest& r);

        void BestRobotSelector();
        
    };

};