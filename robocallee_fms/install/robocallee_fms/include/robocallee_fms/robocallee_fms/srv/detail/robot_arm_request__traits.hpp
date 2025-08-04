// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robocallee_fms:srv/RobotArmRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/robot_arm_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__TRAITS_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robocallee_fms/srv/detail/robot_arm_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robocallee_fms
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotArmRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: shelf_num
  {
    out << "shelf_num: ";
    rosidl_generator_traits::value_to_yaml(msg.shelf_num, out);
    out << ", ";
  }

  // member: pinky_num
  {
    out << "pinky_num: ";
    rosidl_generator_traits::value_to_yaml(msg.pinky_num, out);
    out << ", ";
  }

  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: shoe_info
  {
    out << "shoe_info: ";
    rosidl_generator_traits::value_to_yaml(msg.shoe_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotArmRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: shelf_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shelf_num: ";
    rosidl_generator_traits::value_to_yaml(msg.shelf_num, out);
    out << "\n";
  }

  // member: pinky_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pinky_num: ";
    rosidl_generator_traits::value_to_yaml(msg.pinky_num, out);
    out << "\n";
  }

  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: shoe_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shoe_info: ";
    rosidl_generator_traits::value_to_yaml(msg.shoe_info, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotArmRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robocallee_fms

namespace rosidl_generator_traits
{

[[deprecated("use robocallee_fms::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robocallee_fms::srv::RobotArmRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robocallee_fms::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robocallee_fms::srv::to_yaml() instead")]]
inline std::string to_yaml(const robocallee_fms::srv::RobotArmRequest_Request & msg)
{
  return robocallee_fms::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robocallee_fms::srv::RobotArmRequest_Request>()
{
  return "robocallee_fms::srv::RobotArmRequest_Request";
}

template<>
inline const char * name<robocallee_fms::srv::RobotArmRequest_Request>()
{
  return "robocallee_fms/srv/RobotArmRequest_Request";
}

template<>
struct has_fixed_size<robocallee_fms::srv::RobotArmRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robocallee_fms::srv::RobotArmRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robocallee_fms::srv::RobotArmRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robocallee_fms
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotArmRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotArmRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotArmRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robocallee_fms

namespace rosidl_generator_traits
{

[[deprecated("use robocallee_fms::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robocallee_fms::srv::RobotArmRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robocallee_fms::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robocallee_fms::srv::to_yaml() instead")]]
inline std::string to_yaml(const robocallee_fms::srv::RobotArmRequest_Response & msg)
{
  return robocallee_fms::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robocallee_fms::srv::RobotArmRequest_Response>()
{
  return "robocallee_fms::srv::RobotArmRequest_Response";
}

template<>
inline const char * name<robocallee_fms::srv::RobotArmRequest_Response>()
{
  return "robocallee_fms/srv/RobotArmRequest_Response";
}

template<>
struct has_fixed_size<robocallee_fms::srv::RobotArmRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robocallee_fms::srv::RobotArmRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robocallee_fms::srv::RobotArmRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace robocallee_fms
{

namespace srv
{

inline void to_flow_style_yaml(
  const RobotArmRequest_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotArmRequest_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotArmRequest_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robocallee_fms

namespace rosidl_generator_traits
{

[[deprecated("use robocallee_fms::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robocallee_fms::srv::RobotArmRequest_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  robocallee_fms::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robocallee_fms::srv::to_yaml() instead")]]
inline std::string to_yaml(const robocallee_fms::srv::RobotArmRequest_Event & msg)
{
  return robocallee_fms::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robocallee_fms::srv::RobotArmRequest_Event>()
{
  return "robocallee_fms::srv::RobotArmRequest_Event";
}

template<>
inline const char * name<robocallee_fms::srv::RobotArmRequest_Event>()
{
  return "robocallee_fms/srv/RobotArmRequest_Event";
}

template<>
struct has_fixed_size<robocallee_fms::srv::RobotArmRequest_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robocallee_fms::srv::RobotArmRequest_Event>
  : std::integral_constant<bool, has_bounded_size<robocallee_fms::srv::RobotArmRequest_Request>::value && has_bounded_size<robocallee_fms::srv::RobotArmRequest_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<robocallee_fms::srv::RobotArmRequest_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robocallee_fms::srv::RobotArmRequest>()
{
  return "robocallee_fms::srv::RobotArmRequest";
}

template<>
inline const char * name<robocallee_fms::srv::RobotArmRequest>()
{
  return "robocallee_fms/srv/RobotArmRequest";
}

template<>
struct has_fixed_size<robocallee_fms::srv::RobotArmRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<robocallee_fms::srv::RobotArmRequest_Request>::value &&
    has_fixed_size<robocallee_fms::srv::RobotArmRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<robocallee_fms::srv::RobotArmRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<robocallee_fms::srv::RobotArmRequest_Request>::value &&
    has_bounded_size<robocallee_fms::srv::RobotArmRequest_Response>::value
  >
{
};

template<>
struct is_service<robocallee_fms::srv::RobotArmRequest>
  : std::true_type
{
};

template<>
struct is_service_request<robocallee_fms::srv::RobotArmRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robocallee_fms::srv::RobotArmRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__TRAITS_HPP_
