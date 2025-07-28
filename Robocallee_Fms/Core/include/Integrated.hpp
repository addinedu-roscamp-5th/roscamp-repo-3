#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <mutex>
#include <chrono>
#include <future>
#include <map>
#include <condition_variable>

#include "Logger.hpp"
using namespace Log;

namespace Integrated
{
    #define _LOG_FILE_DIR_ "../../Log_file"

    #define _ROS_NODE_NAME_ std::string("robocallee_fms")

    #define _MAX_EXECUTOR_NUM_ 5

    template<typename T>
    using vec = std::vector<T>;

    using Task = std::function<void()>;

    template<typename T>
    using s_ptr = std::shared_ptr<T>;

    template<typename T>
    using u_ptr = std::unique_ptr<T>;

    template<typename T>
    using w_ptr = std::weak_ptr<T>;

    template<typename T, typename... Args>
    std::shared_ptr<T> make_sptr(Args&&... args)
    {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    template<typename T, typename... Args>
    std::unique_ptr<T> make_uptr(Args&&... args)
    {
        return std::make_unique<T>(std::forward<Args>(args)...);
    }

    enum AmrStep {AmrStep_num};

    enum RobotArmStep {RobotArmStep_num};
};
