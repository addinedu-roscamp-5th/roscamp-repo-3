#pragma once

//Ros 관련
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "Integrated.hpp"
#include "Commondefine.hpp"

namespace interface
{
    class RosInterface : public rclcpp::Node
    {
    private:
        Logger::s_ptr               log_;
        
    public:
        RosInterface(Logger::s_ptr log);
        ~RosInterface();
    };
};