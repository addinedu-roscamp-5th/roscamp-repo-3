#include "RequestManager.hpp"

using namespace Manager;

RequestManager::RequestManager(Integrated::w_ptr<core::ICore> Icore, Logger::s_ptr log)
    :Icore_(Icore), log_(log)
{
    log_->Log(Log::LogLevel::INFO, "RequestManager 객체 생성");
    
}

RequestManager::~RequestManager()
{
    log_->Log(Log::LogLevel::INFO, "RequestManager 객체 소멸");
}

void RequestManager::EnqueueRequest(const Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(request_mtx_);
    request_queue_.push(r);
}

bool RequestManager::PopRequest(Commondefine::GUIRequest& r)
{
    std::lock_guard<std::mutex> lock(request_mtx_);
    if (request_queue_.empty()) return false;
    
    r = request_queue_.front();
    request_queue_.pop();
    return true;

}

void RequestManager::best_pinky_selector()
{

}