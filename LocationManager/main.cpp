

#include "rclcpp/rclcpp.hpp"
#include "include/GwanjeCam.hpp"


#include <iostream>



using namespace std;




int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);  // ROS2 초기화
  // rclcpp::spin(std::make_shared<MinimalPublisher>());


    // GwanjeCam 객체 생성
  std::shared_ptr<GwanjeCam> gwanjeCam = std::make_shared<GwanjeCam>();

    std::thread ros_spin_thread([&]() {
        rclcpp::spin(gwanjeCam->get_node());  // get_node()를 GwanjeCam 클래스에 만들어야 함
    });

  gwanjeCam->monitor();


  rclcpp::shutdown();  // 종료
  


  return 0;

}


