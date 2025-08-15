#include "RobotArmAdapter.hpp"

using namespace Adapter;
using namespace Commondefine;
using namespace Integrated;
using namespace std::chrono_literals;

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

void RobotArmAdapter::checkWorkOnlyOnce()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    //항상 움직이기 전에 새로운 경로가 있는지 확인 하고 출발한다.
    bool timeout = core->waitWorkOnlyOnce(100ms);

    //timeout 되면 다시 checkWorkOnlyOnce() 추가해서 해당 함수가 실행 되도록 한다.
    if(!timeout)
    {
        core->assignTask(request_.robot_id,RobotArmStep::check_work_only_once);
        return;
    }

    core->setWorkOnlyOnce(false);

    //lock 이 풀리면 진짜 움직이는 스텝으로 이동하게 된다.
    core->assignTask(request_.robot_id, request_.command);
}

void RobotArmAdapter::shelfToBuffer()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    //core->getStorage(ContainerType::Shelf, request_);
    core->ArmRequestMakeCall(RobotArm::RobotArm1, 1, request_.amr_id, "shelf_to_buffer");
}

void RobotArmAdapter::bufferToAmr()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    //하드코딩 했음....
    //core->getStorage(ContainerType::Buffer, request_);
    core->ArmRequestMakeCall(RobotArm::RobotArm2, 1, request_.amr_id, "buffer_to_pinky");
}

void RobotArmAdapter::amrToBuffer()
{
    auto core = Icore_.lock();
    if(core == nullptr) 
    {
        log_->Log(Log::LogLevel::ERROR, "core pointer is nullptr");
        return;
    }

    core->ArmRequestMakeCall(RobotArm::RobotArm2, request_.containerIndex, request_.amr_id, "pinky_to_buffer");
}
void RobotArmAdapter::bufferToshelf()
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

    core->ArmRequestMakeCall(RobotArm::RobotArm1, shelf_num, request_.amr_id, "buffer_to_shelf");
}


        
