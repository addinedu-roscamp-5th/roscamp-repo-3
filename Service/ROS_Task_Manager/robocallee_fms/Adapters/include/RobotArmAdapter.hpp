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
        std::vector<bool>shelf_occupied; // 선반이 비어있는지 여부를 저장하는 벡터

    public:
        using u_ptr = Integrated::u_ptr<RobotArmAdapter>;

        RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log);
        ~RobotArmAdapter();



        void arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int robot_id);
        void arm2_buffer_to_pinky(int robot_id);
        void arm2_pinky_to_buffer(int robot_id);
        
        // OCR 여기서 추가해볼까
        
        void arm1_buffer_to_shelf(int robot_id); 

        
    };

};
