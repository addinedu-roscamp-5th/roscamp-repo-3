#include "RobotArmAdapter.hpp"

using namespace Adapter;

RobotArmAdapter::RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore),log_(log)
{
    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 생성");
}

RobotArmAdapter::~RobotArmAdapter()
{
    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 소멸");
}



void RobotArmAdapter::arm1_shelf_to_buffer(Commondefine::shoesproperty shoe, int pinky_num){
// void RobotArmAdapter::arm1_shelf_to_buffer(){


    log_->Log(Log::LogLevel::INFO, "arm1_shelf_to_buffer 호출");



    // ICore 에 함수 추가하고 그 다음 RosInterface에 함수 추가해서 호출해야 함






}
