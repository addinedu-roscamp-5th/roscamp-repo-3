#include "StorageManager.hpp"

using namespace Manager;
using namespace Commondefine;
using namespace Integrated;

StorageManager::StorageManager(Integrated::w_ptr<core::ICore> icore, Log::Logger::s_ptr log)
    :icore_(icore) , log_(log)
{
    //가장 처음에는 true 로 되어 있어서 바로 점유해서 사용할수 있도록 한다.
    criticalSection_.store(true);
    workOnlyOnce_.store(true);

    InitContainer();
}

StorageManager::~StorageManager()
{
}

void StorageManager::InitContainer()
{
    for(int i = 0 ; i < _ARM_SHELF_ ; ++i)
    {
        shelf_info_[i];
    }

    for(int i = 0 ; i < _ARM_BUFFER_ ; ++i)
    {
        buffer_info_[i];
    }

}

void StorageManager::SetWorkOnlyOnce(bool flag)
{
    workOnlyOnce_.store(flag);

    if(flag) workOnlyOnce_cv_.notify_one();
}

bool StorageManager::waitWorkOnlyOnce(std::chrono::milliseconds ms)
{
    std::unique_lock lock(workOnlyOnce_mtx_);

    return workOnlyOnce_cv_.wait_for(lock, ms,[&](){ return workOnlyOnce_.load(); });
}

bool StorageManager::StorageRequest(Commondefine::StorageRequest storage)
{
    {
        std::lock_guard lock(Request_mtx_);
        storageRequest_.push(storage);
    }

    auto core = icore_.lock();
    if(core == nullptr)
    {
        log_->Log(ERROR,"core pointer is nullptr");
        return false;
    }

    core->assignTask(RobotArm::RobotArmNum, RobotArmStep::resolve_Request);
    return true;
}

bool StorageManager::popRequest(Commondefine::StorageRequest& req)
{
    if(storageRequest_.empty()) return false;

    {
        std::lock_guard lock(Request_mtx_);
        req = storageRequest_.front();
        storageRequest_.pop();
    }

    return true;
}

bool StorageManager::resolveRequest()
{
    auto core = icore_.lock();
    if(core == nullptr)
    {
        log_->Log(ERROR,"core pointer is nullptr");
        return false;
    }

    if(storageRequest_.empty()){ return false;}

    Commondefine::StorageRequest req;
    
    //실제로 해야하는 작업은 이미 "req"내부에 있고, 두 로봇팔이 작업중인지 확인하는 루틴으로 들어가고 작업 중이 아니라면
    //바로 실제 수행해야 하는 작업이 된다.
    popRequest(req);
    core->setStorageRequest(req);
    core->assignTask(req.robot_id, RobotArmStep::check_work_only_once);

    // 큐가 빌때 까지 일해야 하기 때문에... 여기 아니면 일다 하고 나면 불러도 된다.
    if(storageRequest_.empty()) 
    {
        //일단 넣어보고 부하가 그리 안걸린다면 빼도된다..
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        core->assignTask(RobotArm::RobotArmNum, RobotArmStep::resolve_Request);
        
        return true;
    }

    return true;
}

void StorageManager::setCriticalSection(bool flag)
{
    criticalSection_.store(flag);

    if(flag)
    {
        Request_cv_.notify_one();
    }
    return;
}

bool StorageManager::waitCriticalSection(std::chrono::milliseconds ms)
{
    //ture 일때 사용 가능이고 false 일때 사용 불가
    std::unique_lock lock(Request_mtx_);

    return Request_cv_.wait_for(lock, ms, [&]() { return criticalSection_.load(); });

}

bool StorageManager::setStorage(Commondefine::ContainerType container, Commondefine::StorageRequest shoes)
{
    std::lock_guard lock(storage_mtx_);

    int index = findEmptyStorage(container);
    if(index == -1)
    {
        log_->Log(INFO,"현재 비어있는 .");
        return false;
    }

    auto& table = (container == ContainerType::Shelf) ? shelf_info_ : buffer_info_;

    auto& s = table[index];
    shoes.container = container;
    shoes.containerIndex = index;

    s.setStorage(shoes.shoes);

    return true;
}

bool StorageManager::getStorage(Commondefine::ContainerType container, Commondefine::StorageRequest& shoes)
{
    std::lock_guard lock(storage_mtx_);
    
    int index = findStorage(container, shoes.shoes);
    if(index == -1) return false;

    auto& table = (container == ContainerType::Shelf) ? shelf_info_ : buffer_info_;

    auto& s = table[index];
    shoes.shoes = s.getStorage();
    
    shoes.container = container;
    shoes.containerIndex = index;

    return true;
}

int StorageManager::findEmptyStorage(Commondefine::ContainerType container)
{
    auto& table = (container == ContainerType::Shelf) ? shelf_info_ : buffer_info_;
    
    auto it = std::find_if(table.begin(), table.end(), [&](const auto& pair)
    {
        return !pair.second.empty();
    });

    return (it == table.end()) ? -1 : static_cast<int>(it->first);
}

int StorageManager::findStorage(Commondefine::ContainerType container, Commondefine::shoesproperty& shoes)
{
    int index = -1;

    auto& table = (container == ContainerType::Shelf) ? shelf_info_ : buffer_info_;

    auto it = std::find_if(table.begin(), table.end(), [&](const auto& pair) 
    { 
        return pair.second.snapshot().size == shoes.size &&
        pair.second.snapshot().model == shoes.model &&
        pair.second.snapshot().color == shoes.color;
    });

    if (it != table.end())
    {
        log_->Log(INFO,"현재 들어온 신발이 선반에 존재 하지 않습니다.");
        return index;
    }

    return it->first;
}
