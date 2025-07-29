#pragma once

#include "ICore.hpp"
#include "Integrated.hpp"

//Ros 관련
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robocallee_fms/srv/shoe_request.hpp" 
#include "robocallee_fms/srv/done_msg.hpp"    

namespace interface
{
    class RosInterface : public rclcpp::Node
    {
    private:
        Logger::s_ptr                           log_;
        Integrated::w_ptr<core::ICore>          Icore_;

        rclcpp::Service<robocallee_fms::srv::ShoeRequest>::SharedPtr    req_service_;
        rclcpp::Service<robocallee_fms::srv::DoneMsg>::SharedPtr        done_service_;

    public:
        using s_ptr = std::shared_ptr<RosInterface>;
        using u_ptr = std::shared_ptr<RosInterface>;
        using w_ptr = std::weak_ptr<RosInterface>;

        RosInterface(Logger::s_ptr log);
        ~RosInterface();

        bool Initialize(Integrated::w_ptr<core::ICore> Icore);

        void CreateRequestService();      
        void CreateDoneService();
    };
};