#include "Integrated.hpp"
#include "Commondefine.hpp"

#include "ICore.hpp"

namespace Adapter
{
    class RobotArmAdapter
    {
    private:
        Integrated::w_ptr<core::ICore>            Icore_;
        Logger::s_ptr                             log_;

    public:
        using u_ptr = Integrated::u_ptr<RobotArmAdapter>;

        RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log);
        ~RobotArmAdapter();



        void arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int pinky_num);
        // void RobotArmAdapter::arm1_shelf_to_buffer();




    };
};