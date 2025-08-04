#include "Integrated.hpp"
#include "Commondefine.hpp"
#include "Core.hpp"

#include "RosInterface.hpp"

#include <iostream>

using namespace Integrated;
using namespace Commondefine;

int main(int argc, char * argv[])
{
    const Logger::s_ptr log_ = make_sptr<Logger>(_LOG_FILE_DIR_, true);

    rclcpp::init(argc, argv);
    log_->Log(INFO,_ROS_NODE_NAME_ + " Node init");
    
    const interface::RosInterface::s_ptr pInterface = make_sptr<interface::RosInterface>(log_);
    const core::Core::s_ptr pCore = make_sptr<core::Core>(log_, pInterface);

    pCore->Initialize();
    pInterface->Initialize(pCore);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pInterface);
    
    log_->Log(INFO,_ROS_NODE_NAME_ + " Node spin");
    executor.spin();

    rclcpp::shutdown();
    log_->Log(INFO,_ROS_NODE_NAME_ + " Node shutdown");

    return 0;
}