// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robocallee_fms:srv/ShoeRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/shoe_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__SHOE_REQUEST__STRUCT_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__SHOE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Request __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShoeRequest_Request_
{
  using Type = ShoeRequest_Request_<ContainerAllocator>;

  explicit ShoeRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->model = "";
      this->size = 0l;
      this->color = "";
      this->x = 0.0f;
      this->y = 0.0f;
      this->customer_id = 0l;
    }
  }

  explicit ShoeRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : requester(_alloc),
    model(_alloc),
    color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->requester = "";
      this->model = "";
      this->size = 0l;
      this->color = "";
      this->x = 0.0f;
      this->y = 0.0f;
      this->customer_id = 0l;
    }
  }

  // field types and members
  using _requester_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _requester_type requester;
  using _model_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _model_type model;
  using _size_type =
    int32_t;
  _size_type size;
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_type color;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _customer_id_type =
    int32_t;
  _customer_id_type customer_id;

  // setters for named parameter idiom
  Type & set__requester(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->requester = _arg;
    return *this;
  }
  Type & set__model(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->model = _arg;
    return *this;
  }
  Type & set__size(
    const int32_t & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__customer_id(
    const int32_t & _arg)
  {
    this->customer_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Request
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Request
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShoeRequest_Request_ & other) const
  {
    if (this->requester != other.requester) {
      return false;
    }
    if (this->model != other.model) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->customer_id != other.customer_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShoeRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShoeRequest_Request_

// alias to use template instance with default allocator
using ShoeRequest_Request =
  robocallee_fms::srv::ShoeRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Response __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShoeRequest_Response_
{
  using Type = ShoeRequest_Response_<ContainerAllocator>;

  explicit ShoeRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_list = 0l;
    }
  }

  explicit ShoeRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->wait_list = 0l;
    }
  }

  // field types and members
  using _wait_list_type =
    int32_t;
  _wait_list_type wait_list;

  // setters for named parameter idiom
  Type & set__wait_list(
    const int32_t & _arg)
  {
    this->wait_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Response
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Response
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShoeRequest_Response_ & other) const
  {
    if (this->wait_list != other.wait_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShoeRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShoeRequest_Response_

// alias to use template instance with default allocator
using ShoeRequest_Response =
  robocallee_fms::srv::ShoeRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Event __attribute__((deprecated))
#else
# define DEPRECATED__robocallee_fms__srv__ShoeRequest_Event __declspec(deprecated)
#endif

namespace robocallee_fms
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShoeRequest_Event_
{
  using Type = ShoeRequest_Event_<ContainerAllocator>;

  explicit ShoeRequest_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ShoeRequest_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::ShoeRequest_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robocallee_fms::srv::ShoeRequest_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Event
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robocallee_fms__srv__ShoeRequest_Event
    std::shared_ptr<robocallee_fms::srv::ShoeRequest_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShoeRequest_Event_ & other) const
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
  bool operator!=(const ShoeRequest_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShoeRequest_Event_

// alias to use template instance with default allocator
using ShoeRequest_Event =
  robocallee_fms::srv::ShoeRequest_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robocallee_fms

namespace robocallee_fms
{

namespace srv
{

struct ShoeRequest
{
  using Request = robocallee_fms::srv::ShoeRequest_Request;
  using Response = robocallee_fms::srv::ShoeRequest_Response;
  using Event = robocallee_fms::srv::ShoeRequest_Event;
};

}  // namespace srv

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__SHOE_REQUEST__STRUCT_HPP_
