#include "Executor.hpp"

using namespace std;
using namespace task;

Executor::Executor(Logger::s_ptr log , int id)
    :log_(log) , id_(id) , isRunning_(true)
{
    ExecutorThread_ = std::thread(&Executor::Run, this);

    log_->Log(INFO , "Executor ID : " + std::to_string(id_) + " initialize done");
}

Executor::~Executor()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_flag_ = true;
    }
    
    cv_.notify_one();
    
    if (ExecutorThread_.joinable())
    {
        ExecutorThread_.join();
    }

    log_->Log(INFO , "Executor ID : " + std::to_string(id_) + " 정상 종료");
}

bool Executor::assignTask(Task task)
{
    if(task == nullptr) return false;

    {
        lock_guard<mutex> lock(mtx_);
    
        task_ = std::move(task);

        currentState = BUSY;
    }

    cv_.notify_one();

    return true;
}

void Executor::Run()
{
    while(isRunning_)
    {
        Task current_Task = nullptr;

        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [&]() { return task_ != nullptr || stop_flag_;});

            if(stop_flag_) return;

            current_Task = std::move(task_);
            task_ = nullptr;
       
        
            if (!current_Task)
            {
                log_->Log(WARN,"현재 작업 Task가 nullptr 입니다.");
                
                currentState = IDEL;
                
                continue;
            }

            current_Task();

            currentState = IDEL;
        
        }
    }

    log_->Log(INFO , "Executor thread 정상 종료");

    return;
}