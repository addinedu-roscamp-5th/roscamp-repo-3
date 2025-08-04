#pragma once

#include <thread>
#include <queue>
#include "Integrated.hpp"
#include "Executor.hpp"

using namespace Integrated;

namespace task
{
    class Dispatcher
    {
    private:
        int                         num_Executors_;
        vec<Executor::u_ptr>        Executors_;
        Logger::s_ptr               log_;

        std::thread                 Dispatcherthread_;
        std::mutex                  mtx_;
        std::condition_variable     cv_;
        bool                        isRunning_;
        bool                        stop_flag_;
        std::queue<Task>            Task_queue_;

        /////////////////////////////////////////pirvate Function
        void Run();
    public:
        using s_ptr = std::shared_ptr<Dispatcher>;
        using u_ptr = std::unique_ptr<Dispatcher>;
        
        Dispatcher(const int ExecutorCount, Logger::s_ptr log);
        ~Dispatcher();


        bool Enqueue(Task task);
    };

};