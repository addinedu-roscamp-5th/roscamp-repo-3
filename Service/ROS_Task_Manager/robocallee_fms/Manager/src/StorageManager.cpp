#include "StorageManager.hpp"

using namespace Manager;

StorageManager::StorageManager(Integrated::w_ptr<core::ICore> icore, Log::Logger::s_ptr log)
    :icore_(icore) , log_(log)
{
    
    pickUpPlace_.store(true);
}

StorageManager::~StorageManager()
{
    
}

bool StorageManager::Enqueue(Commondefine::StorageRequest storage)
{
    {
        std::lock_guard lock(storage_mtx_);

        storageQueue_.push(storage);
    }

    return true;
}