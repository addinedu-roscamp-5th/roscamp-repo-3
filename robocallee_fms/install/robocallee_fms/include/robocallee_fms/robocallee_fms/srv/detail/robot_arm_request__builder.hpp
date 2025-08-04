// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robocallee_fms:srv/RobotArmRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/robot_arm_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__BUILDER_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robocallee_fms/srv/detail/robot_arm_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_RobotArmRequest_Request_shoe_info
{
public:
  explicit Init_RobotArmRequest_Request_shoe_info(::robocallee_fms::srv::RobotArmRequest_Request & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::RobotArmRequest_Request shoe_info(::robocallee_fms::srv::RobotArmRequest_Request::_shoe_info_type arg)
  {
    msg_.shoe_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Request msg_;
};

class Init_RobotArmRequest_Request_action
{
public:
  explicit Init_RobotArmRequest_Request_action(::robocallee_fms::srv::RobotArmRequest_Request & msg)
  : msg_(msg)
  {}
  Init_RobotArmRequest_Request_shoe_info action(::robocallee_fms::srv::RobotArmRequest_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_RobotArmRequest_Request_shoe_info(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Request msg_;
};

class Init_RobotArmRequest_Request_pinky_num
{
public:
  explicit Init_RobotArmRequest_Request_pinky_num(::robocallee_fms::srv::RobotArmRequest_Request & msg)
  : msg_(msg)
  {}
  Init_RobotArmRequest_Request_action pinky_num(::robocallee_fms::srv::RobotArmRequest_Request::_pinky_num_type arg)
  {
    msg_.pinky_num = std::move(arg);
    return Init_RobotArmRequest_Request_action(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Request msg_;
};

class Init_RobotArmRequest_Request_shelf_num
{
public:
  Init_RobotArmRequest_Request_shelf_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotArmRequest_Request_pinky_num shelf_num(::robocallee_fms::srv::RobotArmRequest_Request::_shelf_num_type arg)
  {
    msg_.shelf_num = std::move(arg);
    return Init_RobotArmRequest_Request_pinky_num(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::RobotArmRequest_Request>()
{
  return robocallee_fms::srv::builder::Init_RobotArmRequest_Request_shelf_num();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_RobotArmRequest_Response_success
{
public:
  Init_RobotArmRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robocallee_fms::srv::RobotArmRequest_Response success(::robocallee_fms::srv::RobotArmRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::RobotArmRequest_Response>()
{
  return robocallee_fms::srv::builder::Init_RobotArmRequest_Response_success();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_RobotArmRequest_Event_response
{
public:
  explicit Init_RobotArmRequest_Event_response(::robocallee_fms::srv::RobotArmRequest_Event & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::RobotArmRequest_Event response(::robocallee_fms::srv::RobotArmRequest_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Event msg_;
};

class Init_RobotArmRequest_Event_request
{
public:
  explicit Init_RobotArmRequest_Event_request(::robocallee_fms::srv::RobotArmRequest_Event & msg)
  : msg_(msg)
  {}
  Init_RobotArmRequest_Event_response request(::robocallee_fms::srv::RobotArmRequest_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RobotArmRequest_Event_response(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Event msg_;
};

class Init_RobotArmRequest_Event_info
{
public:
  Init_RobotArmRequest_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotArmRequest_Event_request info(::robocallee_fms::srv::RobotArmRequest_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RobotArmRequest_Event_request(msg_);
  }

private:
  ::robocallee_fms::srv::RobotArmRequest_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::RobotArmRequest_Event>()
{
  return robocallee_fms::srv::builder::Init_RobotArmRequest_Event_info();
}

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__BUILDER_HPP_
