// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robocallee_fms:srv/RobotArmRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/robot_arm_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__STRUCT_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Request __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotArmRequest_Request_
{
  using Type = RobotArmRequest_Request_<ContainerAllocator>;

  explicit RobotArmRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shelf_num = 0l;
      this->pinky_num = 0l;
    }
  }

  explicit RobotArmRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shelf_num = 0l;
      this->pinky_num = 0l;
    }
  }

  // field types and members
  using _shelf_num_type =
    int32_t;
  _shelf_num_type shelf_num;
  using _pinky_num_type =
    int32_t;
  _pinky_num_type pinky_num;

  // setters for named parameter idiom
  Type & set__shelf_num(
    const int32_t & _arg)
  {
    this->shelf_num = _arg;
    return *this;
  }
  Type & set__pinky_num(
    const int32_t & _arg)
  {
    this->pinky_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Request
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Request
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotArmRequest_Request_ & other) const
  {
    if (this->shelf_num != other.shelf_num) {
      return false;
    }
    if (this->pinky_num != other.pinky_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotArmRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotArmRequest_Request_

// alias to use template instance with default allocator
using RobotArmRequest_Request =
  robocallee_fms::srv::RobotArmRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Response __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotArmRequest_Response_
{
  using Type = RobotArmRequest_Response_<ContainerAllocator>;

  explicit RobotArmRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit RobotArmRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Response
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Response
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotArmRequest_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotArmRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotArmRequest_Response_

// alias to use template instance with default allocator
using RobotArmRequest_Response =
  robocallee_fms::srv::RobotArmRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Event __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__RobotArmRequest_Event __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotArmRequest_Event_
{
  using Type = RobotArmRequest_Event_<ContainerAllocator>;

  explicit RobotArmRequest_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit RobotArmRequest_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::RobotArmRequest_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::RobotArmRequest_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Event
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__RobotArmRequest_Event
    std::shared_ptr<robocallee_fms::srv::RobotArmRequest_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotArmRequest_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotArmRequest_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotArmRequest_Event_

// alias to use template instance with default allocator
using RobotArmRequest_Event =
  robocallee_fms::srv::RobotArmRequest_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms

namespace robocallee_fms
{

namespace srv
{

struct RobotArmRequest
{
  using Request = robocallee_fms::srv::RobotArmRequest_Request;
  using Response = robocallee_fms::srv::RobotArmRequest_Response;
  using Event = robocallee_fms::srv::RobotArmRequest_Event;
};

}  // namespace srv

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__ROBOT_ARM_REQUEST__STRUCT_HPP_
