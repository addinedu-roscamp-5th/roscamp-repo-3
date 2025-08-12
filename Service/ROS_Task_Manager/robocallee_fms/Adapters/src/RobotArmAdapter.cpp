#include "RobotArmAdapter.hpp"

using namespace Adapter;
using namespace Commondefine;
using namespace Integrated;

RobotArmAdapter::RobotArmAdapter(Integrated::w_ptr<core::ICore> Icore, int id ,Logger::s_ptr log)
    :Icore_(Icore),log_(log)
{
    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 생성");
}

RobotArmAdapter::~RobotArmAdapter()
{
    log_->Log(Log::LogLevel::INFO,"RobotArmAdapter 객체 소멸");
}


void RobotArmAdapter::setStorageRequest(Commondefine::StorageRequest request)
{
    {
        std::lock_guard lock(request_mtx_);
        request_ = request;

        setState(RobotState::BUSY);
    }
    return;
}

void RobotArmAdapter::shelfToBuffer()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }
    core->getStorage(ContainerType::Shelf, request_);
    core->ArmRequestMakeCall(RobotArm::RobotArm1, request_.containerIndex, request_.robot_id, "shelf_to_buffer");
}

void RobotArmAdapter::bufferToAmr()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }
    core->getStorage(ContainerType::Buffer, request_);
    core->ArmRequestMakeCall(RobotArm::RobotArm2, request_.containerIndex, request_.robot_id, "buffer_to_pinky");
}
void RobotArmAdapter::amrToBuffer()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    core->ArmRequestMakeCall(RobotArm::RobotArm2, request_.containerIndex, request_.robot_id, "pinky_to_buffer");
}
void RobotArmAdapter::bufferToAmr()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    int shelf_num = -1;
    shelf_num = core->findEmptyStorage(ContainerType::Shelf);

    if(shelf_num == -1)
    {
        log_->Log(Log::LogLevel::ERROR, "비어있는 선반이 존재 하지 않습니다.");
        return;
    }

    core->ArmRequestMakeCall(RobotArm::RobotArm1, shelf_num, request_.robot_id, "buffer_to_shelf");
}



        
