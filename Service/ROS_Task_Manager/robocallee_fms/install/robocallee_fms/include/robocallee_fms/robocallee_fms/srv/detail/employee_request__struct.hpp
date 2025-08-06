// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robocallee_fms:srv/EmployeeRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/employee_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__STRUCT_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Request __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EmployeeRequest_Request_
{
  using Type = EmployeeRequest_Request_<ContainerAllocator>;

  explicit EmployeeRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->action = "";
    }
  }

  explicit EmployeeRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : requester(_alloc),
    action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->action = "";
    }
  }

  // field types and members
  using _requester_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _requester_type requester;
  using _action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_type action;

  // setters for named parameter idiom
  Type & set__requester(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->requester = _arg;
    return *this;
  }
  Type & set__action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Request
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Request
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EmployeeRequest_Request_ & other) const
  {
    if (this->requester != other.requester) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    return true;
  }
  bool operator!=(const EmployeeRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EmployeeRequest_Request_

// alias to use template instance with default allocator
using EmployeeRequest_Request =
  robocallee_fms::srv::EmployeeRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Response __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EmployeeRequest_Response_
{
  using Type = EmployeeRequest_Response_<ContainerAllocator>;

  explicit EmployeeRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_list = 0l;
      this->action = "";
      this->success = false;
    }
  }

  explicit EmployeeRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_list = 0l;
      this->action = "";
      this->success = false;
    }
  }

  // field types and members
  using _wait_list_type =
    int32_t;
  _wait_list_type wait_list;
  using _action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_type action;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__wait_list(
    const int32_t & _arg)
  {
    this->wait_list = _arg;
    return *this;
  }
  Type & set__action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Response
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Response
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EmployeeRequest_Response_ & other) const
  {
    if (this->wait_list != other.wait_list) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const EmployeeRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EmployeeRequest_Response_

// alias to use template instance with default allocator
using EmployeeRequest_Response =
  robocallee_fms::srv::EmployeeRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Event __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__EmployeeRequest_Event __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EmployeeRequest_Event_
{
  using Type = EmployeeRequest_Event_<ContainerAllocator>;

  explicit EmployeeRequest_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit EmployeeRequest_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::EmployeeRequest_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::EmployeeRequest_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Event
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__EmployeeRequest_Event
    std::shared_ptr<robocallee_fms::srv::EmployeeRequest_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EmployeeRequest_Event_ & other) const
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
  bool operator!=(const EmployeeRequest_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EmployeeRequest_Event_

// alias to use template instance with default allocator
using EmployeeRequest_Event =
  robocallee_fms::srv::EmployeeRequest_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms

namespace robocallee_fms
{

namespace srv
{

struct EmployeeRequest
{
  using Request = robocallee_fms::srv::EmployeeRequest_Request;
  using Response = robocallee_fms::srv::EmployeeRequest_Response;
  using Event = robocallee_fms::srv::EmployeeRequest_Event;
};

}  // namespace srv

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__STRUCT_HPP_
