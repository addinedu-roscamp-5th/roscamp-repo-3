#include "Dispatcher.hpp"

using namespace task;

Dispatcher::Dispatcher(const int num_Executors , Logger::s_ptr log)
    :num_Executors_(num_Executors) , log_(log) , isRunning_(true)
{
    size_t size = num_Executors_;
    for(size_t i = 0; i < size; ++i)
    {
        Executors_.emplace_back(std::make_unique<Executor>(log_, i));
    }

    Dispatcherthread_ = std::thread(&Dispatcher::Run, this);

    log_->Log(INFO, "Dispatcher initialize done");
}

Dispatcher::~Dispatcher()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_flag_ = true;
    }
    cv_.notify_all();

    if (Dispatcherthread_.joinable())
    {
        Dispatcherthread_.join();
    }

    Executors_.clear();

    log_->Log(INFO , "Dispatcher 정상 종료");
}

void Dispatcher::Run()
{
    while (isRunning_)
    {
        Task task = nullptr;

        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [&]() { return stop_flag_ || !Task_queue_.empty();});

            if (stop_flag_) return;

            if (Task_queue_.empty()) continue;
        
            task = Task_queue_.front();

            if(task == nullptr)
            {
                log_->Log(WARN,"현재 작업 Task가 nullptr 입니다.");
                continue;
            }
            
            size_t size = Executors_.size();

            for(size_t i = 0 ; i < size; ++i)
            {
                if(Executors_[i]->GetState() == IDEL)
                {
                    Task_queue_.pop();
                    Executors_[i]->assignTask(task);
                    
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    return;
}

bool Dispatcher::Enqueue(Task task)
{
    if(task == nullptr) return false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        Task_queue_.push(task);
    }

    cv_.notify_one();
    
    return true;
}