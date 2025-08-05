#pragma once

#include "ICore.hpp"
#include "Integrated.hpp"
#include "Commondefine.hpp"

// ROS2 기본
#include "rclcpp/rclcpp.hpp"

// 서비스·클라이언트
#include "robocallee_fms/srv/shoe_request.hpp"
#include "robocallee_fms/srv/done_msg.hpp"
#include "robocallee_fms/srv/robot_arm_request.hpp"

// LM 위치 토픽 메시지 타입
#include "std_msgs/msg/float32_multi_array.hpp"
// Aruco PoseStamped 토픽 메시지 타입
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <mutex>
#include <vector>
#include <string>

namespace interface 
{

// 로봇 개수 (인덱스 타입과 일치하도록 int 사용)
static constexpr int NUM_PINKY = 3;

using LmPoseMsg      = std_msgs::msg::Float32MultiArray;
using ReqServiceType = robocallee_fms::srv::ShoeRequest;
using DoneServiceType= robocallee_fms::srv::DoneMsg;
using ArmServiceType = robocallee_fms::srv::RobotArmRequest;

class RosInterface : public rclcpp::Node {
public:
    using s_ptr = std::shared_ptr<RosInterface>;
    using w_ptr = std::weak_ptr<RosInterface>;

    explicit RosInterface(Logger::s_ptr log);
    ~RosInterface() override;

    bool Initialize(Integrated::w_ptr<core::ICore> Icore);

    // 외부(Core)에서 호출될 때
    void arm1_send_request(int shelf_num, int pinky_num);

private:
    Logger::s_ptr                   log_;
    Integrated::w_ptr<core::ICore>  Icore_;

    // 서비스
    rclcpp::Service<ReqServiceType>::SharedPtr  req_service_;
    rclcpp::Service<DoneServiceType>::SharedPtr done_service_;

    // 팔 클라이언트
    rclcpp::Client<ArmServiceType>::SharedPtr   arm1_client_;
    void cbArmService(rclcpp::Client<ArmServiceType>::SharedFuture future);

    // LM 위치 구독
    std::vector< rclcpp::Subscription<LmPoseMsg>::SharedPtr > pose_subs_;
    // Aruco Pose 구독
    std::vector< rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr > aruco_subs_;

    // 받은 좌표 저장
    mutable std::mutex                       pose_mutex_;
    std::vector<Commondefine::pose2f>        last_poses_;

    
    void lmArrayCallback(const LmPoseMsg::ConstSharedPtr & msg, int pinky_id);

    void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg,
                            int pinky_id);

    void onArucoPose(int pinky_id, const Commondefine::pose2f &p);

    // 요청·완료 서비스 콜백
    void cbRequestService(const std::shared_ptr<ReqServiceType::Request>  request,
                          std::shared_ptr<ReqServiceType::Response>       response);
    void cbDoneService(const std::shared_ptr<DoneServiceType::Request> request,
                       std::shared_ptr<DoneServiceType::Response>      response);
};

}
