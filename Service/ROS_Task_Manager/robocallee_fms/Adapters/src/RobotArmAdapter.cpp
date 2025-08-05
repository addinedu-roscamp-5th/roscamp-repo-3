#include "RobotArmAdapter.hpp"

using namespace Adapter;

RobotArmAdapter::RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore),log_(log)
{
    shelf_info.resize(10);


    Commondefine::shoesproperty example_shoe;
    example_shoe.size = 260;
    example_shoe.model = "Nike";
    example_shoe.color = "White";
    
    shelf_info[5] = example_shoe;

    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 생성");
}

RobotArmAdapter::~RobotArmAdapter()
{
    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 소멸");
}



/* 선반 위치를 다음과 같이 정의. 

1  2  3 
4  5  6  
7  8  9

9개 뿐이니 그냥 for문으로 찾는 걸로

   typedef struct shoesproperty
    {
        int             size;
        std::string     model;
        std::string     color;
    }shoesproperty;


*/
void RobotArmAdapter::arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int pinky_num){

    log_->Log(Log::LogLevel::INFO, "arm1_shelf_to_buffer 호출");
    
    
    // 찾는 신발이 놓여있는 선반 번호 찾기

    int shelf_num = -1;
    for (int i=1 ; i<=9 ; ++i) {
        if (shelf_info[i].size == shoe.size && 
            shelf_info[i].model == shoe.model && 
            shelf_info[i].color == shoe.color) {
            shelf_num = i;
            break;
        }
    }

    if(shelf_num == -1) {
        log_->Log(Log::LogLevel::ERROR, "Arm1: 해당 신발 정보를 찾을 수 없습니다.");
        return;
    }
    log_->Log(Log::LogLevel::INFO, "선반 번호 " + shelf_num );

    

    // ICore의 ArmRequestMakeCall() 호출하고  ArmRequestMakeCall()는 또 RosInterface의 arm1_send_request()를 호출
    if (auto core = Icore_.lock()) {
        core->ArmRequestMakeCall(1, shelf_num, pinky_num);
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }

}
