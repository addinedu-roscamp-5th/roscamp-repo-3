#include "Integrated.hpp"
#include "Core.hpp"
#include "RosInterface.hpp"

using namespace Integrated;
using namespace interface;
using namespace core;

int main(int argc, char** argv)
{
    Log::Logger::s_ptr Log = make_sptr<Log::Logger>(_LOG_FILE_DIR_, true);


    const s_ptr<interface::RosInterface> pnterface = make_sptr<interface::RosInterface>(Log);
    const u_ptr<core::Core> pCore = make_uptr<core::Core>(Log, pnterface);

    rclcpp::init(argc, argv); 
    rclcpp::spin(pnterface);
    
    rclcpp::shutdown();
    return 0;
}
