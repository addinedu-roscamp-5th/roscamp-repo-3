#include "RobotArmAdapter.hpp"

using namespace Adapter;

RobotArmAdapter::RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore),log_(log)
{
    shelf_info.resize(10);
    shelf_occupied.resize(10, false); // 0번 인덱스는 사용하지 않음, 1~9번 인덱스에 선반 정보 저장, 초기에는 모두 비어있다고 가정

    // 선반 정보 초기화.
    // 일단 하드 코딩

    Commondefine::shoesproperty example_shoe;
    example_shoe.size = 260;
    example_shoe.model = "Nike";
    example_shoe.color = "White";
    
    shelf_info[5] = example_shoe;
    shelf_occupied[5] = true; // 5번 선반에 신발이 있다고 가정


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


void RobotArmAdapter::arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int robot_id){

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
    log_->Log(Log::LogLevel::INFO, "꺼낼 선반 번호 " + std::to_string(shelf_num));


    // 선반 비어있음으로 표시
    shelf_info[shelf_num].size = -1;
    shelf_info[shelf_num].model = "";
    shelf_info[shelf_num].color = "";
    shelf_occupied[shelf_num] = false; 



    // ICore의 ArmRequestMakeCall() 호출하고  ArmRequestMakeCall()는 또 RosInterface의 arm1_send_request()를 호출
    if (auto core = Icore_.lock()) {
        core->ArmRequestMakeCall(1, shelf_num, robot_id, "shelf_to_buffer");
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }


    // 후속 작업 없으니 호출 안 함

}





void RobotArmAdapter::arm2_buffer_to_pinky(int robot_id)
{
    log_->Log(Log::LogLevel::INFO, "arm2_buffer_to_pinky 호출");
    int shelf_num = -1;


    // ICore의 ArmRequestMakeCall() 호출하고  ArmRequestMakeCall()는 또 RosInterface의 arm1_send_request()를 호출
    if (auto core = Icore_.lock()) {
        core->ArmRequestMakeCall(2, shelf_num, robot_id, "buffer_to_pinky");
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }

        // 후속 작업 필요 시 호출

 
    // AMR Adapter의 함수 불러주기     robot_id만 넘겨주면 될듯

    // if (auto core = Icore_.lock()) {
    //     core->SetAmrNextStep(Commondefine::AmrAdapter::MoveTo_dest2 , robot_id );
    // } else {
    //     log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    // }


}


void RobotArmAdapter::arm2_pinky_to_buffer(int robot_id)
{

    log_->Log(Log::LogLevel::INFO, "arm2_pinky_to_buffer 호출");
    
    int shelf_num = -1;

    // ICore의 ArmRequestMakeCall() 호출하고  ArmRequestMakeCall()는 또 RosInterface의 arm1_send_request()를 호출
    if (auto core = Icore_.lock()) {
        core->ArmRequestMakeCall(2, shelf_num, robot_id, "pinky_to_buffer");
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }

    // 후속 작업 필요 시 호출

    // AMR Adapter의 함수 불러주기   robot_id만 넘겨주면 될듯
    // if (auto core = Icore_.lock()) {
    //     core->SetAmrNextStep(Commondefine::AmrAdapter::MoveTo_dest3 , robot_id );
    // } else {
    //     log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    // }


    // arm1_buffer_to_shelf(shoe, robot_id) 불러주기
    Commondefine::shoesproperty dummy_shoe;
    if (auto core = Icore_.lock() ) {
        core->SetRobotArmNextStep( Commondefine::RobotArmStep::buffer_to_shelf , dummy_shoe, robot_id);
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }


}





// 이 타이밍에서 OCR 시도
void RobotArmAdapter::arm1_buffer_to_shelf(int robot_id){

    log_->Log(Log::LogLevel::INFO, "arm1_buffer_to_shelf 호출");
    
    
    // 비어있는 선반 번호 찾기
    int shelf_num = -1;


    for(int i=1; i<=9; ++i) {
        if (!shelf_occupied[i]) { // 비어있는 선반 찾기
            shelf_num = i;
            break;
        }
    }

    //
    if(shelf_num == -1) {
        log_->Log(Log::LogLevel::ERROR, "Arm1: 비어있는 선반이 없습니다.");
        return;
    }
    log_->Log(Log::LogLevel::INFO, "놓을 선반 번호 " + std::to_string(shelf_num));



    // ICore의 ArmRequestMakeCall() 호출하고  ArmRequestMakeCall()는 또 RosInterface의 arm1_send_request()를 호출
    if (auto core = Icore_.lock()) {
        core->ArmRequestMakeCall(1, shelf_num, robot_id, "buffer_to_shelf");
    } else {
        log_->Log(Log::LogLevel::ERROR, "ICore 호출 실패");
    }

    // response 오면 RosInterface::cbArmService() 호출 -> Core::UpdateShelfInfo() 호출 통해 서랍 정보 업뎃

    // shelf_occupied[shelf_num] = true; 
    // shelf_info[shelf_num] = Commondefine::shoesproperty{260, "Nike", "White"}; // 예시로 하드코딩된 신발 정보. 실제론 OCR 결과로 받아서 처리해야 함

}



        
