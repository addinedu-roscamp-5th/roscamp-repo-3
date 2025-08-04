#include "RosInterface.hpp"

using namespace interface;

RosInterface::RosInterface(Logger::s_ptr log)
    :rclcpp::Node(_ROS_NODE_NAME_), log_(log)
{

}

RosInterface::~RosInterface()
{
    
}