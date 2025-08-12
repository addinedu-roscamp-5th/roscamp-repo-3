#pragma once
#include "ICore.hpp"
#include "Integrated.hpp"
#include "Commondefine.hpp"
#include "ClientWrapper.hpp"
// ROS2 기본
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// 서비스·클라이언트
#include "robocallee_fms/srv/customer_request.hpp"
#include "robocallee_fms/srv/employee_request.hpp"
#include "robocallee_fms/srv/robot_arm_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// LM 위치 토픽 메시지 타입
#include "std_msgs/msg/float32_multi_array.hpp"
// Aruco PoseStamped 토픽 메시지 타입
#include "robocallee_fms/msg/aruco_pose_array.hpp"
#include "robocallee_fms/msg/aruco_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>
#include <vector>
#include <string>

namespace interface
{
    using LmPoseMsg                 = std_msgs::msg::Float32MultiArray;
    using CustomerServiceType       = robocallee_fms::srv::CustomerRequest;
    using EmployeeServiceType       = robocallee_fms::srv::EmployeeRequest;
    using ArmServiceType            = robocallee_fms::srv::RobotArmRequest;
    using ArucoPoseArray            = robocallee_fms::msg::ArucoPoseArray;
    using ArucoPose                 = robocallee_fms::msg::ArucoPose;

    class RosInterface : public rclcpp::Node
    {
    private:
        Logger::s_ptr                                                   log_;
        Integrated::w_ptr<core::ICore>                                  Icore_;
        // 서비스
        rclcpp::Service<robocallee_fms::srv::CustomerRequest>::SharedPtr        customer_service_;
        rclcpp::Service<robocallee_fms::srv::EmployeeRequest>::SharedPtr        employee_service_;
        // Arm 클라이언트
        std::vector<rclcpp::Client<ArmServiceType>::SharedPtr>          arm_clients_;
        // Aruco Pose 구독
        rclcpp::Subscription<ArucoPoseArray>::SharedPtr  aurco_array_sub_;
        // poseStamped publish
        std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> nav_goal_pubs_;
        std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>         odom_pubs_;
        // 받은 좌표 저장
        std::mutex                                                      pose_mutex_;
        //서버와 연결될때 까지 대기
        std::mutex                                                      client_mutex_;
        std::condition_variable                                         client_cv_;
        std::map<std::string , CW::ClientWrapperBase::s_ptr>            wait_clients_;
        std::thread                                                     server_wait_thread_;
        bool                                                            server_wait_is_Running;
        void async_server_wait();
        template<typename ServiceT>
        void AddWaitClient(const std::string& name, std::shared_ptr<rclcpp::Client<ServiceT>> client);

    public:
        using s_ptr = std::shared_ptr<RosInterface>;
        using w_ptr = std::weak_ptr<RosInterface>;
        explicit RosInterface(Logger::s_ptr log);
        ~RosInterface() override;
        bool Initialize(Integrated::w_ptr<core::ICore> Icore);
        // 외부(Core)에서 호출될 때
        void arm1_send_request(int shelf_num, int robot_id , std::string action );
        void arm2_send_request(int robot_id, std::string action );
        void cbArmService(rclcpp::Client<ArmServiceType>::SharedFuture future);
        void cbarucoPoseArray(const ArucoPoseArray::ConstSharedPtr & msg);
        void publishNavGoal(int idx, const Commondefine::Position wp);
        // 요청·완료 서비스 콜백
        void cbCustomerRequest(const std::shared_ptr<CustomerServiceType::Request> request,
            std::shared_ptr<CustomerServiceType::Response> response);
        void cbEmployeeRequest(const std::shared_ptr<EmployeeServiceType::Request> request,
            std::shared_ptr<EmployeeServiceType::Response> response);
    };

    template<typename ServiceT>
    void RosInterface::AddWaitClient(const std::string& name, std::shared_ptr<rclcpp::Client<ServiceT>> client)
    {
        auto wrapper = std::make_shared<CW::ClientWrapper<ServiceT>>(name, client);
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            wait_clients_.insert(std::make_pair(name, std::static_pointer_cast<CW::ClientWrapperBase>(wrapper)));
        }
        client_cv_.notify_one();
    }
}