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

        std::vector<Commondefine::shoesproperty> shelf_info; // 0번 인덱스는 사용하지 않음, 1~9번 인덱스에 선반 정보 저장

    public:
        using u_ptr = Integrated::u_ptr<RobotArmAdapter>;

        RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log);
        ~RobotArmAdapter();



        void arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int pinky_num);



    };
};