#pragma once

#include "ICore.hpp"
#include "Integrated.hpp"

//Ros 관련
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robocallee_fms/srv/shoe_request.hpp" 
#include "robocallee_fms/srv/done_msg.hpp"    

#include "robocallee_fms/srv/robot_arm_request.hpp"    


namespace interface
{
    using ReqServiceType = robocallee_fms::srv::ShoeRequest;    
    using DoneServiceType = robocallee_fms::srv::DoneMsg;
    using ArmServiceType = robocallee_fms::srv::RobotArmRequest;


    class RosInterface : public rclcpp::Node
    {
    private:
        Logger::s_ptr                           log_;
        Integrated::w_ptr<core::ICore>          Icore_;

        rclcpp::Service<robocallee_fms::srv::ShoeRequest>::SharedPtr    req_service_;
        rclcpp::Service<robocallee_fms::srv::DoneMsg>::SharedPtr        done_service_;


        rclcpp::Client<robocallee_fms::srv::RobotArmRequest>::SharedPtr        arm1_client_;
        void cbArmService(rclcpp::Client<ArmServiceType>::SharedFuture future);





        void cbRequestService(const std::shared_ptr<ReqServiceType::Request> request, std::shared_ptr<ReqServiceType::Response> response);
        void cbDoneService(const std::shared_ptr<DoneServiceType::Request> request,std::shared_ptr<DoneServiceType::Response> response);





    public:
        using s_ptr = std::shared_ptr<RosInterface>;
        using u_ptr = std::shared_ptr<RosInterface>;
        using w_ptr = std::weak_ptr<RosInterface>;

        RosInterface(Logger::s_ptr log);
        ~RosInterface();

        bool Initialize(Integrated::w_ptr<core::ICore> Icore);

        void arm1_send_request(int shelf_num, int pinky_num );


    };

};


