#include "StorageManager.hpp"

using namespace Manager;
using namespace Commondefine;
using namespace Integrated;

StorageManager::StorageManager(Integrated::w_ptr<core::ICore> icore, Log::Logger::s_ptr log)
    :icore_(icore) , log_(log)
{
    //가장 처음에는 true 로 되어 있어서 바로 점유해서 사용할수 있도록 한다.
    criticalSection_.store(true);

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

    core->assignTask(RobotArmStep::resolve_Request);
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
    
    popRequest(req);
    core->setStorageRequest(req);
    core->assignTask(req.command);

    // 큐가 빌때 까지 일해야 하기 때문에... 여기 아니면 일다 하고 나면 불러도 된다.
    if(storageRequest_.empty()) 
    {
        //일단 넣어보고 부하가 그리 안걸린다면 빼도된다..
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        core->assignTask(RobotArmStep::resolve_Request);
        return true;
    }
}

bool StorageManager::checkCriticalSection()
{
    auto core = icore_.lock();
    if(core == nullptr)
    {
        log_->Log(ERROR,"core pointer is nullptr");
        return false;
    }

    //pick up place 가 누군가 사용하고 있을때는 계속 계속 찾는다.
    if(!criticalSection_.load())
    {
        //너무 빠른 속도로 폴링을 하게 되면, 변화가 없는데 무분별하게 thread queue에 쌓일 수 있기 때문에 약간의 sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        core->assignTask(RobotArmStep::check_critical_section);
        return false;
    }

    setCriticalSection(false);

    //이제 진짜 다음 스텝으로 넘어 간다.
    core->assignTask(RobotArmStep::buffer_to_Amr);

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

void StorageManager::waitCriticalSection()
{
    //ture 일때 사용 가능이고 false 일때 사용 불가
    if(criticalSection_.load()) return;

    {
        std::unique_lock lock(Request_mtx_);
        Request_cv_.wait(lock,[&]() { return criticalSection_.load(); });
    }

    return;
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
