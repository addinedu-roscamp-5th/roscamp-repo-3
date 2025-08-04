#pragma once

#include "Integrated.hpp"
#include <thread>

using namespace Integrated;

namespace task
{
    enum ExecutorState { IDEL = 0, BUSY };
    class Executor
    {
    private:
        int                         id_;
        std::thread                 ExecutorThread_;
        std::mutex                  mtx_;
        std::condition_variable     cv_;
        Task                        task_;
        ExecutorState               currentState;
        bool                        isRunning_;
        bool                        stop_flag_;
        
        Logger::s_ptr               log_;
        
        /////////////////////////////////////////pirvate Function
        void Run();
    
    public:
        using s_ptr = std::shared_ptr<Executor>;
        using u_ptr = std::unique_ptr<Executor>;
        using w_ptr = std::weak_ptr<Executor>;

        Executor(Logger::s_ptr log , int id);
        ~Executor();

        ExecutorState GetState() const {return currentState;}

        bool assignTask(Task task);
    };
};
