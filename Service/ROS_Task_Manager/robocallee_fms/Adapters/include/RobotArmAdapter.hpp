#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "ICore.hpp"
#include <vector>


namespace Adapter
{
    class RobotArmAdapter
    {
    private:
        Integrated::w_ptr<core::ICore>            Icore_;
        Logger::s_ptr                             log_;

        std::atomic<Commondefine::RobotState>     state_;
        Commondefine::StorageRequest              request_;
        std::mutex                                request_mtx_;
    public:
        using u_ptr = Integrated::u_ptr<RobotArmAdapter>;

        RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, int id, Logger::s_ptr log);
        ~RobotArmAdapter();

        void setState(Commondefine::RobotState state){ state_.store(state); }
        Commondefine::RobotState getState() { return state_.load(); }

        void setStorageRequest(Commondefine::StorageRequest request);

        void shelfToBuffer();

        void bufferToAmr();

        void amrToBuffer();

        void bufferToAmr();
    };

};
